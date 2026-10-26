#include "main.h"
#include "fdcan.h"
#include "My_CAN.h"

/* CubeMX 生成的 fdcan.h 里已经 extern 声明了 hfdcan1/2/3，这里不用重复声明 */

HAL_StatusTypeDef BSP_CAN_Init(FDCAN_HandleTypeDef *hfdcan,
                               const CAN_Filter_t *filters,
                               uint8_t count)
{
    FDCAN_FilterTypeDef hw;
    uint8_t i;

    if (hfdcan == NULL) {
        return HAL_ERROR;
    }
    /* filters 和 count 必须成对出现 */
    if ((count > 0U) && (filters == NULL)) {
        return HAL_ERROR;
    }
     /*越界会静默写坏*/ 
    if (count > hfdcan->Init.StdFiltersNbr) {
        return HAL_ERROR;
    }

    /* 1) 全局滤波器：没有任何滤波器命中的帧一律不收 */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 2) 逐条写入硬件滤波器列表 */
    for (i = 0U; i < count; i++) {
        hw.IdType       = FDCAN_STANDARD_ID;
        hw.FilterIndex  = i;
        hw.FilterType   = FDCAN_FILTER_MASK;
        hw.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        hw.FilterID1    = filters[i].id;    /* HAL 内部负责左移，传原始 11 位 ID */
        hw.FilterID2    = filters[i].mask;

        if (HAL_FDCAN_ConfigFilter(hfdcan, &hw) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    /* 3) 打开 Rx FIFO0 新消息中断 */
    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 4) 启动 */
    return HAL_FDCAN_Start(hfdcan);
}

HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *hfdcan,
                               uint16_t id,
                               const uint8_t *data,
                               uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx;

    if ((hfdcan == NULL) || (data == NULL) || (len > 8U)) {
        return HAL_ERROR;
    }

    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = len;              /* 经典 CAN：DLC 值 0~8 恰好等于字节数 */
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;    /* 经典 CAN，不做变速 */
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0U;

    /* 发送队列满时 HAL 返回 HAL_ERROR，调用方自行决定要不要重试 */
    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx, (uint8_t *)data);
}

/* HAL 的 FIFO0 接收中断回调 → 取出报文 → 转交给应用层 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
        return;
    }

    BSP_CAN_OnReceive(hfdcan,
                      (uint16_t)rx.Identifier,
                      data,
                      (uint8_t)rx.DataLength);
}

/* 默认空实现。应用层在任意一个 .c 里重新实现同名函数即可覆盖 */
__weak void BSP_CAN_OnReceive(FDCAN_HandleTypeDef *hfdcan,
                              uint16_t id,
                              const uint8_t *data,
                              uint8_t len)
{
    (void)hfdcan;
    (void)id;
    (void)data;
    (void)len;
}
