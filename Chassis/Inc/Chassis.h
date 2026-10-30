/** @brief 底盘软件配置入口；运动学接口仍由各轮系库提供。 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "Chassis_Conf.h"

#if CHASSIS_ENABLE_MECANUM
#include "Mecanum.h"
#elif CHASSIS_ENABLE_OMNIWHEEL
#include "OmniWheel.h"
#elif CHASSIS_ENABLE_SWERVE
#include "Swerve.h"

/* 独立的四轮状态，按左前、右前、左后、右后排列。 */
extern Swerve_Module_t g_swerve_modules[SWERVE_MODULE_COUNT];
#endif

/**
 * 按 Chassis_Conf.h 加载软件初始参数，重复调用会复位运行状态。
 * 只初始化软件数据，不读取编码器、不回零、不启动电机、PID 或 CAN。
 * 舵轮尺寸和初始舵角需要由使用者配置，控制前需填入真实校准反馈。
 */
void Chassis_Init(void);

#endif /* CHASSIS_H */
