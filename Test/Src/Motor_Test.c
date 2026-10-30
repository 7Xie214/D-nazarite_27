#include "Motor_Test.h"
#include "My_CAN.h"

HAL_StatusTypeDef Test_MotorCurrent(FDCAN_HandleTypeDef *hfdcan,
                                    uint8_t motor_id,
                                    int16_t current)
{
    uint8_t data[8] = {0};
    uint8_t slot;
    uint16_t raw;
    uint16_t can_id;

    if ((motor_id < 1u) || (motor_id > 8u)) {
        return HAL_ERROR;
    }

    slot = (uint8_t)((motor_id - 1u) % 4u);
    can_id = (motor_id <= 4u) ? 0x200u : 0x1FFu;
    raw = (uint16_t)current;
    data[slot * 2u] = (uint8_t)(raw >> 8u);
    data[slot * 2u + 1u] = (uint8_t)(raw & 0xFFu);

    return BSP_CAN_Send(hfdcan, can_id, data, 8u);
}

HAL_StatusTypeDef Test_GM6020Voltage(FDCAN_HandleTypeDef *hfdcan,
                                    uint8_t motor_id,
                                    int16_t voltage)
{
    uint8_t data[8] = {0};
    uint8_t slot;
    uint16_t raw;
    uint16_t can_id;

    if ((motor_id < 1u) || (motor_id > 7u)) {
        return HAL_ERROR;
    }

    slot = (uint8_t)((motor_id - 1u) % 4u);
    can_id = (motor_id <= 4u) ? 0x1FFu : 0x2FFu;
    raw = (uint16_t)voltage;
    data[slot * 2u] = (uint8_t)(raw >> 8u);
    data[slot * 2u + 1u] = (uint8_t)(raw & 0xFFu);

    return BSP_CAN_Send(hfdcan, can_id, data, 8u);
}
