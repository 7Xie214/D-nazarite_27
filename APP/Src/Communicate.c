#include "Communicate.h"

#include "My_USART.h"
#include "usart.h"          /* huart10 */

#include "FreeRTOS.h"       /* configTICK_RATE_HZ */
#include "cmsis_os2.h"      /* osKernelGetTickCount / osWaitForever */

#include <string.h>

/* ===========================================================================
 * 解包分四层，从上往下：
 *
 *   ① frame_feed      字节流   → 完整帧         （状态机，跨调用保持）
 *   ② frame_is_valid  完整帧   → 可信帧         （帧尾 + CRC 把关）
 *   ③ frame_decode    可信帧   → Bluer_Frame_t  （按协议取字段）
 *   ④ frame_dispatch  结构体   → 应用动作
 *
 * 前一层不知道后一层的存在，所以换协议只动 ③④，拆包和校验一个字不用改。
 *
 * ---------------------------------------------------------------------------
 * 多实例：整个文件里【没有一处】文件级 static 的解析状态。
 * 半帧缓冲、长度、诊断计数全在 Bluer_Ctx_t 里，由调用方持有。
 * 所以同一个 Communicate_Task 可以同时跑在好几个串口上，互不干扰。
 *
 * 【维护红线】以后往这个文件加东西时，凡是"会随数据变化"的量，
 * 一律加到 Bluer_Ctx_t 里去，别再写 static 局部状态 ——
 * 那正是多路串口会互相踩的根源。
 * =========================================================================== */

/* 帧间隔换算成 tick。
 * 用 configTICK_RATE_HZ 而不是硬编码 1000：将来 CubeMX 里把 tick 改成别的值，
 * 这里自动跟着变，不会变成"标称 100ms 实际 1s"这种难查的错。 */
#define BLUER_GAP_TICKS  ((uint32_t)BLUER_FRAME_GAP_MS * (uint32_t)configTICK_RATE_HZ / 1000u)

/* ====================== 第一路蓝牙的实例（挂在 huart10） ======================
 * 非 static：Keil 的 Watch 窗口 / .map 里能直接找到，调 CRC 错误很方便。
 * 加第二路就在这里再定义一份，并把 huart 换成对应的句柄。 */
Bluer_Ctx_t g_bluer_bt1 = { .huart = &huart10 };

/* ============================ CRC-16/CCITT-FALSE ============================
 * 参数：poly=0x1021  init=0xFFFF  refin=false  refout=false  xorout=0x0000
 *
 * 逐位实现不查表：省 512 字节 Flash，115200 下算 7 个字节只要几微秒。
 * 如果你的发送端用的是别的变体（比如 MODBUS 的反射多项式 0xA001，
 * 或者干脆是简单累加和），只改这个函数内部，调用处完全不用动。
 *
 * 纯函数、无状态 → 多实例共用一份，天然安全。 */
static uint16_t crc16(const uint8_t *p, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint8_t  i;

    while (len > 0u) {
        crc ^= (uint16_t)((uint16_t)(*p) << 8);
        p++;
        len--;

        for (i = 0u; i < 8u; i++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ============================== ② 校验 ==============================
 * 要记诊断计数，所以得知道是哪一路 → 收 ctx。 */
static int frame_is_valid(Bluer_Ctx_t *ctx, const uint8_t *f)
{
    uint16_t crc_calc;
    uint16_t crc_recv;

    /* 帧头由拆包状态机保证过了，这里再查一次是防御性写法 */
    if (f[0] != BLUER_HEAD) {
        return 0;
    }

    if (f[BLUER_FRAME_LEN - 1u] != BLUER_TAIL) {
        ctx->tail_err++;
        return 0;
    }

    /* 小端：低字节在前 */
    crc_recv = (uint16_t)((uint16_t)f[1u + BLUER_PAYLOAD_LEN] |
                          ((uint16_t)f[2u + BLUER_PAYLOAD_LEN] << 8));
    /* CRC 覆盖 帧头 + 载荷 = 偏移 0..(1+PAYLOAD-1)，共 1+PAYLOAD 字节 */
    crc_calc = crc16(f, (uint16_t)(1u + BLUER_PAYLOAD_LEN));

    if (crc_calc != crc_recv) {
        ctx->crc_err++;
        return 0;
    }
    return 1;
}

/* 从缓冲区按小端取一个 uint16 */
static uint16_t rd_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ============================== ③ 解包 ==============================
 * 逐字段从线上字节里取，不做整体 memcpy。
 * 原因：结构体字段顺序和线上顺序不一致；且小端/大端由这里显式决定，
 * 不受编译器和 CPU 影响。 */
static void frame_decode(const uint8_t *f, Bluer_Frame_t *out)
{
    const uint8_t *d = &f[1];       /* 载荷起点（跳过 1 字节帧头） */

    out->region = d[BLUER_OFF_REGION];
    out->mode   = d[BLUER_OFF_MODE];
    out->repeat = d[BLUER_OFF_REPEAT];
    out->data   = rd_u16_le(&d[BLUER_OFF_DATA]);
    out->remain = rd_u16_le(&d[BLUER_OFF_REMAIN]);
}

/* ============================== ④ 派发 ==============================
 * 这里是任务上下文（不是中断），可以放心调用阻塞 API、操作 CAN 等。
 *
 * 收 ctx 而不是只收 msg：多路蓝牙时业务需要知道"消息是从哪一路来的"，
 * 比如：
 *     if (ctx->huart == &huart10) { ... 前舱 ... }
 *     else                        { ... 后舱 ... }
 * 只收 msg 的话这个信息在派发层就丢了。 */
static void frame_dispatch(Bluer_Ctx_t *ctx, const Bluer_Frame_t *msg)
{
    (void)ctx;
    (void)msg;


    /* TODO: 按业务处理，例如
     *   if (msg->region == 1) { ... }
     */
    if(ctx->huart == &huart10){
        huart_10_Task(msg);
    }


}

/* ============================== ① 拆包实现 ==============================
 * 逐字节推进，收满 BLUER_FRAME_LEN 才尝试解一帧。
 * 所有状态都在 ctx 里，调用方给哪份 ctx 就在哪份上推进。 */
static void frame_feed(Bluer_Ctx_t *ctx, const uint8_t *p, uint16_t n)
{
    uint16_t i;
    Bluer_Frame_t msg;

    for (i = 0u; i < n; i++) {

        /* 还没进帧：只认帧头。
         * 帧与帧之间的所有字节都是垃圾，静默丢掉 —— 这里不计错，
         * 否则链路空闲时每一个字节都会让计数器爆掉，反而看不清真问题。 */
        if ((ctx->len == 0u) && (p[i] != BLUER_HEAD)) {
            continue;
        }

        ctx->buf[ctx->len] = p[i];
        ctx->len++;

        if (ctx->len >= BLUER_FRAME_LEN) {
            if (frame_is_valid(ctx, ctx->buf)) {
                frame_decode(ctx->buf, &msg);
                frame_dispatch(ctx, &msg);
            }
            /* 无论成败都回零：定长协议下，失败就整帧丢弃重新找帧头。
             * 不做"从帧内再找帧头"的复杂重同步 —— 帧头+CRC+帧尾三重把关，
             * 误同步概率已经很低，多写那些代码不划算。 */
            ctx->len = 0u;
        }
    }
}

/* ============================ 对外：喂一段字节 ============================ */
void Communicate_Parse(Bluer_Ctx_t *ctx, const uint8_t *data, uint16_t n)
{
    if ((ctx == NULL) || (data == NULL) || (n == 0u)) {
        return;
    }
    frame_feed(ctx, data, n);
}

/* ============================== 组帧发送 ============================== */
HAL_StatusTypeDef Communicate_SendFrame(UART_HandleTypeDef *huart,
                                        const uint8_t *payload,
                                        uint16_t len,
                                        uint32_t timeout_ms)
{
    uint8_t  frame[BLUER_FRAME_LEN];
    uint16_t crc;

    if ((huart == NULL) || (payload == NULL) || (len != BLUER_PAYLOAD_LEN)) {
        return HAL_ERROR;           /* 定长协议：载荷长度必须正好 */
    }

    frame[0] = BLUER_HEAD;
    memcpy(&frame[1], payload, BLUER_PAYLOAD_LEN);

    crc = crc16(frame, (uint16_t)(1u + BLUER_PAYLOAD_LEN));
    frame[1u + BLUER_PAYLOAD_LEN] = (uint8_t)(crc & 0xFFu);         /* 小端：低字节在前 */
    frame[2u + BLUER_PAYLOAD_LEN] = (uint8_t)((crc >> 8) & 0xFFu);

    frame[BLUER_FRAME_LEN - 1u] = BLUER_TAIL;

    return BSP_USART_Send(huart, frame, (uint16_t)BLUER_FRAME_LEN, timeout_ms);
}

/* ================================ 任务 ================================
 * 通用解析任务：argument 传 Bluer_Ctx_t*。
 * 同一个函数体，几路蓝牙就跑几份，栈各自独立、状态各自独立。 */
void Communicate_Task(void *argument)
{
    Bluer_Ctx_t *ctx = (Bluer_Ctx_t *)argument;
    uint8_t  rx[64];
    uint16_t n;
    uint32_t now;

    /* 参数没配好（忘了传、或 ctx 里 huart 是空的）属于编码错误，
     * 这时候最忌讳空转烧 CPU。挂在一个长延时里，留着调试器进来查。 */
    if ((ctx == NULL) || (ctx->huart == NULL)) {
        for (;;) {
            osDelay(1000u);
        }
    }

    /* BSP 内部创建信号量，放在任务入口，避免裸机测试时屏蔽 HAL tick。 */
    if (BSP_USART_Init(ctx->huart) != HAL_OK) {
        for (;;) {
            osDelay(1000u);
        }
    }

    for (;;) {

        /* 读到至少 1 字节才返回。用 osWaitForever 而不是固定超时：
         * 链路空闲时任务就一直在信号量上睡着，0% CPU，也不会每秒醒 10 次。
         * （BSP 内部把 osWaitForever 夹到 0xFFFFFFFE tick ≈ 49 天，等效于永远） */
        n = BSP_USART_Recv(ctx->huart, rx, (uint16_t)sizeof(rx), osWaitForever);

        if (n == 0u) {
            continue;       /* 防御性：正常路径下不会走到 */
        }

        /* 帧间隔超时在这里判，不再借接收超时兼职。
         * 好处：超时值和时间参数解耦 —— 想把间隔调成 200ms，只动
         * BLUER_FRAME_GAP_MS 一个宏，不用去想"接收那边还等不等得及"。
         *
         * 判据：距离上一批字节已经过了 GAP，说明上一帧收了一半就断了
         * （发送端复位、丢包、接触不良）。必须丢掉这半帧 ——
         * 留着的话下一次的数据会拼在它后面，白等一整帧的长度才发现校验错，
         * 等于多丢一帧。 */
        now = osKernelGetTickCount();
        if ((ctx->len != 0u) && ((now - ctx->last_tick) > BLUER_GAP_TICKS)) {
            ctx->len = 0u;
            ctx->sync_err++;
        }
        ctx->last_tick = now;

        /* 这里只做拆包/解包，不阻塞。真正耗时的动作在 frame_dispatch 里 */
        frame_feed(ctx, rx, n);
    }
}



/* ================= CubeMX 生成的那个任务的入口（第一路） ================= */
void Communicate_BluetoothTask(void)
{
    Communicate_Task(&g_bluer_bt1);
}


// #define upstairs_mode 0x00

void huart_10_Task(const Bluer_Frame_t *msg){
    uint8_t mode=msg->mode;
    switch (mode){
        case upstairs_mode:

        break;
        case downstairs_mode:
        
        break;
        
        default:
        break;
    }
}
