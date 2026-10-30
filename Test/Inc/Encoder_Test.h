#ifndef ENCODER_TEST_H
#define ENCODER_TEST_H

#include "main.h"

/* 按电机 ID 升序保存四台 GM6020 的原始角度（0~8191），同 ID 时第一路在前。 */
extern volatile uint16_t g_encoder_calibration[4];
extern volatile uint8_t g_encoder_calibration_mask; /* 0x0F：四台均已收到 */

/* 前两个 ID 属于 can1，后两个属于 can2；ID 为 1~7，两条总线可有相同 ID。
 * HAL_OK 仅表示开始采集；每台只保存开始后第一帧，直到再次调用本函数才重采。
 * 两条 CAN 必须事先由 BSP 启动，滤波器需放行 0x204+ID 的反馈报文。
 * 校准时暂停正常电机控制，避免与现有 3508/2006 的报文冲突。
 * 本函数不启动电机、不修改 CAN 配置、不回零、不写 Flash。 */
HAL_StatusTypeDef Test_EncoderCalibrate(FDCAN_HandleTypeDef *can1,
                                       FDCAN_HandleTypeDef *can2,
                                       uint8_t id1, uint8_t id2,
                                       uint8_t id3, uint8_t id4);

/* 从现有 CAN 接收回调转入，仅提取匹配电机的反馈角度。 */
void Test_EncoderOnCanFrame(FDCAN_HandleTypeDef *hfdcan,
                            uint16_t can_id,
                            const uint8_t *data,
                            uint8_t len);

#endif /* ENCODER_TEST_H */
