#include "Encoder_Test.h"

#include <stddef.h>

volatile uint16_t g_encoder_calibration[4];
volatile uint8_t g_encoder_calibration_mask;

typedef struct {
    FDCAN_HandleTypeDef *bus;
    uint8_t id;
} EncoderTarget_t;

static EncoderTarget_t s_targets[4];
static volatile uint8_t s_collecting;

HAL_StatusTypeDef Test_EncoderCalibrate(FDCAN_HandleTypeDef *can1,
                                       FDCAN_HandleTypeDef *can2,
                                       uint8_t id1, uint8_t id2,
                                       uint8_t id3, uint8_t id4)
{
    EncoderTarget_t targets[4] = {
        {can1, id1}, {can1, id2}, {can2, id3}, {can2, id4}
    };
    uint32_t irq_mask;
    uint8_t i, j;

    if ((can1 == NULL) || (can2 == NULL) || (can1 == can2) ||
        (id1 < 1u) || (id1 > 7u) || (id2 < 1u) || (id2 > 7u) ||
        (id3 < 1u) || (id3 > 7u) || (id4 < 1u) || (id4 > 7u) ||
        (id1 == id2) || (id3 == id4)) {
        return HAL_ERROR;
    }

    /* ID 与总线一起稳定排序，同 ID 的 CAN1 条目保持在 CAN2 前面。 */
    for (i = 1u; i < 4u; i++) {
        const EncoderTarget_t target = targets[i];
        j = i;
        while ((j > 0u) && (targets[j - 1u].id > target.id)) {
            targets[j] = targets[j - 1u];
            j--;
        }
        targets[j] = target;
    }

    /* 重采时只保护一次上下文替换，保留调用者原来的中断开关状态。 */
    irq_mask = __get_PRIMASK();
    __disable_irq();
    for (i = 0u; i < 4u; i++) {
        s_targets[i] = targets[i];
        g_encoder_calibration[i] = 0u;
    }
    g_encoder_calibration_mask = 0u;
    s_collecting = 1u;
    __set_PRIMASK(irq_mask);

    return HAL_OK;
}

void Test_EncoderOnCanFrame(FDCAN_HandleTypeDef *hfdcan,
                            uint16_t can_id,
                            const uint8_t *data,
                            uint8_t len)
{
    uint16_t angle;
    uint8_t i;

    if ((s_collecting == 0u) || (data == NULL) || (len < 7u)) {
        return;
    }
    angle = (uint16_t)(((uint16_t)data[0] << 8u) | (uint16_t)data[1]);
    if (angle > 8191u) {
        return;
    }

    for (i = 0u; i < 4u; i++) {
        const uint8_t bit = (uint8_t)(1u << i);
        if ((s_targets[i].bus == hfdcan) &&
            (can_id == (uint16_t)(0x204u + s_targets[i].id)) &&
            ((g_encoder_calibration_mask & bit) == 0u)) {
            g_encoder_calibration[i] = angle;
            g_encoder_calibration_mask = (uint8_t)(g_encoder_calibration_mask | bit);
            break;
        }
    }
}
