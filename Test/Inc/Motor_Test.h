#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "main.h"

/* 向指定 CAN 总线的 1~8 号 C620/C610 电调发送一次原始电流命令（不是安培）。
 * M3508/C620 有效量程 ±16384，M2006/C610 为 ±10000，由调用方保证。
 * 同组其他三台电机按 0 发送，不要与 MotorApp 同时发送同一组控制帧。
 * 需要持续出力时，由调用方周期调用；本函数不创建任务、不绑定 PID。 */
HAL_StatusTypeDef Test_MotorCurrent(FDCAN_HandleTypeDef *hfdcan,
                                    uint8_t motor_id,
                                    int16_t current);

/* 向指定 CAN 总线的 1~7 号 GM6020 发送一次原始电压命令。
 * 电压模式量程 ±30000（不是伏特或电流），由调用方保证。
 * ID1~4 使用 0x1FF，ID5~7 使用 0x2FF，同组其他槽位发 0。
 * 0x1FF 也控制 C620/C610 ID5~8，测试时不要与其他组帧发送同时使用。
 * 需要持续出力时，由调用方周期调用；不创建任务、不绑定 PID。 */
HAL_StatusTypeDef Test_GM6020Voltage(FDCAN_HandleTypeDef *hfdcan,
                                    uint8_t motor_id,
                                    int16_t voltage);

#endif /* MOTOR_TEST_H */
