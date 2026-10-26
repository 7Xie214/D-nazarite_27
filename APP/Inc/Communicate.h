#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include "main.h"

/* ===========================================================================
 * 通信 / 协议解析层。
 *
 * 任务主体放在这一层，而不是写在 Core/Src/freertos.c 里 ——
 * freertos.c 由 CubeMX 生成，重新生成会覆盖非 USER CODE 区。
 * 那边只保留一行转发调用，我们的代码就永远安全。
 *
 * 本层支持多实例：一路蓝牙 = 一个串口 + 一份 Bluer_Ctx_t。
 * 所有解析状态都装在 Bluer_Ctx_t 里，不占文件级 static，
 * 所以多路同时工作不会互相污染。
 * =========================================================================== */

/* ======================= 帧格式（按你的实际协议改） =======================
 *
 * 线上字节序（小端）：
 *
 *   偏移  0     1       2       3  4     5  6      7  8     9
 *        ┌────┬───────┬───────┬──────┬──────┬───────┬────┐
 *        │0xAB│region │repeat │ data │remain│ CRC16 │0xFB│
 *        └────┴───────┴───────┴──────┴──────┴───────┴────┘
 *        帧头   1B      1B      2B     2B      2B     帧尾
 *
 *   CRC 覆盖偏移 0~6（帧头 + 载荷，共 7 字节），不含 CRC 本身和帧尾。
 * ====================================================================== */

#define BLUER_HEAD        0xABu     /* 帧头 */
#define BLUER_TAIL        0xFBu     /* 帧尾 */

#define BLUER_CRC_LEN     2u
/* 载荷 = region(1) +mode(1) + repeat(1) + data(2) + remain(2) */
#define BLUER_PAYLOAD_LEN 7u

/* 总长 = 帧头1 + 载荷 + CRC2 + 帧尾1。改上面三个宏会自动跟着变 */
#define BLUER_FRAME_LEN   (1u + BLUER_PAYLOAD_LEN + BLUER_CRC_LEN + 1u)   /* = 11 */

/* 载荷内部各字段的相对偏移。单独定义出来，是为了避免代码里散落魔法数字 */
#define BLUER_OFF_REGION  0u
#define BLUER_OFF_MODE    1u
#define BLUER_OFF_REPEAT  2u
#define BLUER_OFF_DATA    3u
#define BLUER_OFF_REMAIN  5u

/* 帧间隔超时：两批字节之间隔了这么久，就认为上一帧收了一半断掉了，
 * 丢弃半截帧重新同步。太小会误杀慢速发送端，太大恢复慢。 */
#define BLUER_FRAME_GAP_MS  100u

/* ============================== 解包结果 ==============================
 * 注意：字段顺序和"线上顺序"无关，也不需要一致。
 * 线上原始字节躺在 ctx->buf 里逐字节对应，这个结构体只是"翻译"的结果。
 * 千万别把串口缓冲 memcpy 进这个结构体 —— 会错位。 */
typedef struct {
    uint8_t  region;
    uint8_t  mode;
    uint8_t  repeat;
    uint16_t data;
    uint16_t remain;
} Bluer_Frame_t;

/* ========================= 一路蓝牙的完整上下文 =========================
 *
 * 【关键】所有会随数据变化的量都在这里面。
 * 千万不要再往 Communicate.c 里加文件级 static 的解析状态 ——
 * 那样两路蓝牙会共用同一份，半帧互相灌进同一个 buf，CRC 全错。
 *
 * 用法（每个实例一行，零初始化即可）：
 *     static Bluer_Ctx_t s_bt1 = { .huart = &huart10 };
 *     static Bluer_Ctx_t s_bt2 = { .huart = &huart1  };
 * ===================================================================== */
typedef struct {
    UART_HandleTypeDef *huart;              /* 这一路挂在哪个串口 */

    /* ---- 拆包状态 ---- */
    uint8_t  buf[BLUER_FRAME_LEN];          /* 半帧暂存 */
    uint16_t len;                           /* 已收字节数；0 = 不在帧内 */
    uint32_t last_tick;                     /* 上一批字节到达的时刻（帧间隔计时） */

    /* ---- 诊断计数（调试时放 Watch 窗口直接看）----
     *   sync_err 一直涨 → 半截帧总在超时，发送端可能一帧分得比 100ms 还散
     *   crc_err  在涨   → 能同步但内容被破坏：干扰、丢字节、或 CRC 算法不一致
     *   tail_err 在涨   → 长度对但帧尾不对：BLUER_FRAME_LEN 和发送端不一致 */
    uint32_t sync_err;
    uint32_t crc_err;
    uint32_t tail_err;
} Bluer_Ctx_t;

/* ================================ API ================================= */

/* 把一段字节流喂给某一路解析器：拆包 → 校验 → 解包 → 派发，全程不阻塞。
 * 不依赖任何文件级状态，所以可以给任意多个实例并发调用。 */
void Communicate_Parse(Bluer_Ctx_t *ctx, const uint8_t *data, uint16_t n);

/* 通用的解析任务：argument 传 Bluer_Ctx_t*。
 * 多个蓝牙模块共用这一个函数，靠 argument 区分实例。
 * 自己开新任务时用它：
 *     osThreadNew(Communicate_Task, &s_bt2, &attr); */
void Communicate_Task(void *argument);

/* CubeMX 生成的 Bluer_Da_analyz 任务的入口，内部就是第一路实例。
 * 额外再加的蓝牙模块，走上面的 Communicate_Task。 */
void Communicate_BluetoothTask(void);

/* 组一帧发出去（第一条参数是串口，不是 ctx —— 发送不需要解析状态）。
 * 内部自己算帧头/CRC/帧尾，你只管给载荷。
 * payload 布局：[region][repeat][data_L][data_H][remain_L][remain_H] */
HAL_StatusTypeDef Communicate_SendFrame(UART_HandleTypeDef *huart,
                                        const uint8_t *payload,
                                        uint16_t len,
                                        uint32_t timeout_ms);

typedef enum{
    upstairs_mode =0,
    downstairs_mode =1,
}huart_10_mode;

void huart_10_Task(const Bluer_Frame_t *msg);

#endif
