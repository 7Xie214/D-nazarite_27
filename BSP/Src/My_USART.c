#include "main.h"
#include "My_USART.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

/* ===========================================================================
 * 每个串口实例一份状态，用静态数组管理 —— 不碰只有 15KB 的 FreeRTOS 堆。
 *
 * 这里刻意不 include "usart.h"：BSP 不该认识 huart10 这种具体实例，
 * 所有函数都靠传入的 handle 查表。这就是"多个串口共用一套函数"的落点。
 * =========================================================================== */
typedef struct {
    UART_HandleTypeDef *huart;              /* 主键；NULL 表示这个槽还空着 */

    /* ---- 接收：DMA 直接写进 rxbuf，rxbuf 本身就是环形缓冲 ---- */
    uint8_t  rxbuf[BSP_USART_RX_SIZE];
    volatile uint16_t rx_head;              /* 已写入的总字节数（ISR 独写） */
    volatile uint16_t rx_tail;              /* 已取走的字节数（任务写；ISR 溢出时也写） */
    uint16_t dma_pos_last;                  /* 上一次事件时 DMA 的绝对写位置 */
    volatile uint32_t rx_overrun;           /* 累计丢弃字节数（诊断） */
    volatile uint32_t rx_rearm;             /* 错误重装次数（诊断） */
    volatile uint16_t rx_err;               /* 最近一次 HAL 错误码（诊断） */
    osSemaphoreId_t   rx_sem;               /* "有数据可读"闩锁 */
    StaticSemaphore_t rx_sem_cb;

    /* ---- 发送 ---- */
    uint8_t  txbuf[BSP_USART_TX_SIZE];      /* 暂存，隔开调用方缓冲区的生命周期 */
    volatile uint8_t tx_inflight;           /* 1 = 有一笔 DMA 发送在途 */
    osSemaphoreId_t   tx_sem;               /* "发送完成" */
    StaticSemaphore_t tx_sem_cb;
} BSP_USART_Ctx_t;

static BSP_USART_Ctx_t g_usart_ctx[BSP_USART_MAX_INSTANCES];

/* ============================ 内部小工具 ============================ */

static BSP_USART_Ctx_t *usart_ctx_find(UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0U; i < BSP_USART_MAX_INSTANCES; i++) {
        if (g_usart_ctx[i].huart == huart) {
            return &g_usart_ctx[i];
        }
    }
    return NULL;
}

/* 毫秒 → RTOS tick。向上取整，避免 1ms 在 1000Hz 下被算成 0 导致非预期立即返回 */
static uint32_t ms_to_ticks(uint32_t ms)
{
    uint32_t freq = osKernelGetTickFreq();
    uint32_t ticks;

    if (ms == 0U) {
        return 0U;
    }
    ticks = (uint32_t)(((unsigned long long)ms * (unsigned long long)freq + 999ULL) / 1000ULL);
    if (ticks == 0U) {
        ticks = 1U;
    }
    if (ticks > 0xFFFFFFFEU) {
        ticks = 0xFFFFFFFEU;    
    }
    return ticks;
}

/* ============================== 初始化 ============================== */

HAL_StatusTypeDef BSP_USART_Init(UART_HandleTypeDef *huart)
{
    BSP_USART_Ctx_t *ctx;
    osSemaphoreAttr_t attr;
    uint8_t i;

    if (huart == NULL) {
        return HAL_ERROR;
    }

    ctx = usart_ctx_find(huart);

    if (ctx == NULL) {
        /* 找一个空槽 */
        for (i = 0U; i < BSP_USART_MAX_INSTANCES; i++) {
            if (g_usart_ctx[i].huart == NULL) {
                ctx = &g_usart_ctx[i];
                break;
            }
        }
        if (ctx == NULL) {
            return HAL_ERROR;               /* 池满，把 BSP_USART_MAX_INSTANCES 调大 */
        }

        /* 用静态控制块创建信号量，不消耗 FreeRTOS 堆 */
        attr.name      = NULL;
        attr.attr_bits = 0U;
        attr.cb_mem    = &ctx->rx_sem_cb;
        attr.cb_size   = sizeof(StaticSemaphore_t);
        ctx->rx_sem    = osSemaphoreNew(1U, 0U, &attr);

        attr.cb_mem    = &ctx->tx_sem_cb;
        ctx->tx_sem    = osSemaphoreNew(1U, 0U, &attr);

        if ((ctx->rx_sem == NULL) || (ctx->tx_sem == NULL)) {
            return HAL_ERROR;
        }

        ctx->rx_head      = 0U;
        ctx->rx_tail      = 0U;
        ctx->dma_pos_last = 0U;
        ctx->rx_overrun   = 0U;
        ctx->rx_rearm     = 0U;
        ctx->rx_err       = 0U;
        ctx->tx_inflight  = 0U;

        ctx->huart = huart;                 /* 放到最后：初始化的中途不"上线" */
    }

    /* 启动循环 DMA 接收。只调用这一次，之后 DMA 永不停机 ——
     * 这就是选 Circular 模式的意义：不用在每次收完后重启，没有丢字节窗口 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, ctx->rxbuf, BSP_USART_RX_SIZE) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ================================ 发送 ================================ */

HAL_StatusTypeDef BSP_USART_SendDMA(UART_HandleTypeDef *huart,
                                    const uint8_t *data,
                                    uint16_t len)
{
    BSP_USART_Ctx_t *ctx = usart_ctx_find(huart);

    if ((ctx == NULL) || (data == NULL) || (len == 0U) || (len > BSP_USART_TX_SIZE)) {
        return HAL_ERROR;
    }

    /* 检查 + 置位要原子：否则两个任务可能同时通过检查，一起用同一块 txbuf */
    taskENTER_CRITICAL();
    if (ctx->tx_inflight != 0U) {
        taskEXIT_CRITICAL();
        return HAL_BUSY;                    /* 同一实例同时只允许一笔发送在途 */
    }
    ctx->tx_inflight = 1U;
    taskEXIT_CRITICAL();

    /* 拷进内部暂存：HAL 的 DMA 发送是异步的，直接传调用方的指针，
     * 栈上的局部数组会在传输结束前就失效 */
    memcpy(ctx->txbuf, data, len);

    if (HAL_UART_Transmit_DMA(huart, ctx->txbuf, len) != HAL_OK) {
        ctx->tx_inflight = 0U;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef BSP_USART_Send(UART_HandleTypeDef *huart,
                                 const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout_ms)
{
    BSP_USART_Ctx_t *ctx = usart_ctx_find(huart);

    if (ctx == NULL) {
        return HAL_ERROR;
    }

    if (BSP_USART_SendDMA(huart, data, len) != HAL_OK) {
        return HAL_BUSY;
    }

    if (osSemaphoreAcquire(ctx->tx_sem, ms_to_ticks(timeout_ms)) != osOK) {
        /* 超时：主动中止，否则 DMA 还在发而 txbuf 会被下一次发送覆写 */
        (void)HAL_UART_AbortTransmit(huart);
        ctx->tx_inflight = 0U;
        /* 排空可能残留的令牌：若刚好在超时那一瞬间发完，TxCplt 已经释放过信号量，
         * 不清掉的话下一次发送会不等就直接返回 */
        (void)osSemaphoreAcquire(ctx->tx_sem, 0U);
        return HAL_TIMEOUT;
    }
    return HAL_OK;
}

/* ================================ 接收 ================================ */

uint16_t BSP_USART_Recv(UART_HandleTypeDef *huart,
                        uint8_t *buf,
                        uint16_t len,
                        uint32_t timeout_ms)
{
    BSP_USART_Ctx_t *ctx = usart_ctx_find(huart);
    uint32_t ticks, t0, elapsed, remain;
    uint16_t n, used, i;

    if ((ctx == NULL) || (buf == NULL) || (len == 0U)) {
        return 0U;
    }

    ticks  = ms_to_ticks(timeout_ms);
    remain = ticks;
    t0     = osKernelGetTickCount();

    for (;;) {
        /* 先试着读，读不到再睡。
         * 反过来的话，数据可能在"查完"和"睡下"之间到达 → 丢唤醒。
         * 临界区是必须的：rx_tail 不只任务在写，ISR 在缓冲区溢出时也会写，
         * 不屏蔽中断的话 ISR 可能在拷贝途中把它挪走，把 DMA 即将覆写的字节交给任务。 */
        taskENTER_CRITICAL();
        used = (uint16_t)(ctx->rx_head - ctx->rx_tail);
        n    = (used < len) ? used : len;
        for (i = 0U; i < n; i++) {
            buf[i] = ctx->rxbuf[(uint16_t)(ctx->rx_tail + i) & (BSP_USART_RX_SIZE - 1U)];
        }
        ctx->rx_tail += n;
        taskEXIT_CRITICAL();

        if (n > 0U) {
            return n;
        }
        if (timeout_ms == 0U) {
            return 0U;                      /* 非阻塞：没数据立刻返回 */
        }
        if (osSemaphoreAcquire(ctx->rx_sem, remain) != osOK) {
            return 0U;                      /* 超时 */
        }
        /* 重算剩余时间，避免反复空唤醒把总等待时间拖长 */
        elapsed = osKernelGetTickCount() - t0;
        if (elapsed >= ticks) {
            return 0U;
        }
        remain = ticks - elapsed;
    }
}

uint16_t BSP_USART_Available(UART_HandleTypeDef *huart)
{
    BSP_USART_Ctx_t *ctx = usart_ctx_find(huart);
    uint16_t used;

    if (ctx == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();
    used = (uint16_t)(ctx->rx_head - ctx->rx_tail);
    taskEXIT_CRITICAL();

    return used;
}

uint32_t BSP_USART_RxOverrunCount(UART_HandleTypeDef *huart)
{
    BSP_USART_Ctx_t *ctx = usart_ctx_find(huart);

    return (ctx == NULL) ? 0U : ctx->rx_overrun;
}

HAL_StatusTypeDef BSP_USART_FlushRx(UART_HandleTypeDef *huart)
{
    BSP_USART_Ctx_t *ctx = usart_ctx_find(huart);

    if (ctx == NULL) {
        return HAL_ERROR;
    }

    taskENTER_CRITICAL();
    ctx->rx_tail = ctx->rx_head;
    taskEXIT_CRITICAL();

    return HAL_OK;
}

/* ========================== HAL 回调（全局，按 handle 分发）============ */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BSP_USART_Ctx_t *ctx = usart_ctx_find(huart);
    uint16_t pos, n, used, drop;

    if (ctx == NULL) {
        return;                             /* 不是我们管的串口，忽略 */
    }

    /* HAL 传来的 Size 是 DMA 在缓冲里的"绝对写位置"（0..RX_SIZE），
     * 不是本次新增的字节数。注意 Size==RX_SIZE 实际表示位置 0
     * （计数器回绕到满，此时写指针刚回到起点）。 */
    pos = (Size >= BSP_USART_RX_SIZE) ? 0U : Size;

    /* 模运算求增量。缓冲区是 2 的幂，所以用掩码代替取模 */
    n = (uint16_t)((uint16_t)(pos - ctx->dma_pos_last) & (BSP_USART_RX_SIZE - 1U));
    ctx->dma_pos_last = pos;

    if (n == 0U) {
        return;                             /* 没带新数据的空转 IDLE */
    }

    /* 应用还没读走就绕回来了 → 只能丢弃最旧的数据（DMA 已经覆写，救不回来） */
    used = (uint16_t)(ctx->rx_head - ctx->rx_tail);
    if ((uint32_t)used + (uint32_t)n > BSP_USART_RX_SIZE) {
        drop = (uint16_t)((uint32_t)used + (uint32_t)n - BSP_USART_RX_SIZE);
        ctx->rx_tail    += drop;
        ctx->rx_overrun += drop;
    }

    ctx->rx_head += n;
    (void)osSemaphoreRelease(ctx->rx_sem);  /* ISR 安全（本工程中断优先级允许） */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_USART_Ctx_t *ctx = usart_ctx_find(huart);

    if (ctx == NULL) {
        return;
    }

    ctx->tx_inflight = 0U;
    (void)osSemaphoreRelease(ctx->tx_sem);
}

/* 这个回调不是可选的：DMA 接收模式下 USART_CR3_DMAR 是置位的，任何 RX 错误
 * （溢出/帧错/噪声/校验/DMA）都会走 HAL 的"阻塞分支"，内部把 RxState 打回
 * READY、清掉 DMAR 和 IDLE 中断、中止 DMA —— 接收不会自己恢复。
 * 不重装的话这个串口就永远收不到东西了。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_USART_Ctx_t *ctx = usart_ctx_find(huart);

    if (ctx == NULL) {
        return;
    }

    ctx->rx_err = (uint16_t)huart->ErrorCode;
    ctx->rx_rearm++;

    /* 发送中途挂掉：TxCplt 不会再来了，得手动放行正在等它的任务 */
    if (ctx->tx_inflight != 0U) {
        ctx->tx_inflight = 0U;
        (void)osSemaphoreRelease(ctx->tx_sem);
    }

    /* HAL 重装接收时会从缓冲偏移 0 重新开始写，所以索引必须一起清零，
     * 否则环形缓冲的"物理位置 ↔ 逻辑字节"对应关系就错位了。 */
    ctx->rx_head      = 0U;
    ctx->rx_tail      = 0U;
    ctx->dma_pos_last = 0U;

    (void)HAL_UARTEx_ReceiveToIdle_DMA(huart, ctx->rxbuf, BSP_USART_RX_SIZE);
}
