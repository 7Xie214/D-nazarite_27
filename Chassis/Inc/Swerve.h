/**
 * @brief 四舵轮逆运动学：底盘速度 -> 轮组舵角 / 轮缘线速度。
 *
 * 坐标约定：+x 朝前，+y 朝左，+wz 逆时针。单位统一为 m、m/s、rad、rad/s。
 * 本层不依赖电机型号、HAL、PID 或 CAN。编码器零偏/方向、电流方向由应用层
 * 适配；传入的实际舵角和输出的目标舵角必须处在同一机械坐标系。
 */
#ifndef CHASSIS_SWERVE_H
#define CHASSIS_SWERVE_H

#include <stddef.h>
#include <stdint.h>

#define SWERVE_MODULE_COUNT    4u
#define SWERVE_SPEED_EPS_MPS   1.0e-4f

/* Swerve_Module_t 数组必须按此顺序排列。 */
typedef enum {
    SWERVE_FRONT_LEFT = 0,
    SWERVE_FRONT_RIGHT,
    SWERVE_REAR_LEFT,
    SWERVE_REAR_RIGHT
} Swerve_WheelIndex_t;

typedef enum {
    SWERVE_OK = 0,
    SWERVE_ERR_PARAM = -1
} Swerve_Status_t;

/* 舵向配置。仅保存，解算中不应用这些硬件方向系数。 */
typedef struct {
    uint16_t zero_offset;       /* 编码器零偏计数，由应用层解释其分辨率 */
    int8_t encoder_direction;  /* 安装方向：+1 / -1 */
    int8_t electirc_direction; /* 保留原字段拼写，安装方向：+1 / -1 */
} rudder_direction_t;

/* 驱动轮配置。R 只用于轮侧 RPM 换算，不参与轮缘线速度解算。 */
typedef struct {
    int8_t electric_direction;
    int8_t encoder_direction;
    float R;                   /* 轮子半径，m */
} steering_gear_t;

typedef struct {
    rudder_direction_t rudder_direction;
    steering_gear_t steering_gear;

    float V_x;                  /* 本次底盘指令，m/s；成功解算后更新 */
    float V_y;                  /* 本次底盘指令，m/s；成功解算后更新 */
    float W_z;                  /* 本次底盘指令，rad/s；成功解算后更新 */
    float target_angle_rad;     /* 连续目标舵角，rad；零速时保留上次值 */
    float target_speed_mps;     /* 有正负号的轮缘线速度，m/s */
    float half_wheel_base;      /* 到中心的纵向距离绝对值，m（半轴距） */
    float half_track_width;     /* 到中心的横向距离绝对值，m（半轮距） */
    float actual_angle_rad;     /* 已校准的实际机械舵角，rad，可跨圈累计 */
} Swerve_Module_t;

/**
 * 同时解算四个轮组；module_count 必须为 SWERVE_MODULE_COUNT。
 * modules 指向调用方持有的四元素数组，调用前刷新各轮 actual_angle_rad。
 * 首次使用时调用方需把 target_angle_rad 设为希望静止保持的舵角。
 *
 * max_wheel_speed_mps 必须为正数；超限时对四轮速度等比例缩放。
 * 假定舵向机构可连续旋转：选择距实际舵角不超过 90 度的等效目标，必要时
 * 反转轮速。目标角不会强制归一化到 [-pi, pi)，以便位置环直接计算误差。
 * 轮速幅值 <= SWERVE_SPEED_EPS_MPS 时停驱动、保留上次目标舵角。
 *
 * 空指针、非四轮、非有限数值、负几何尺寸或计算溢出返回 SWERVE_ERR_PARAM，
 * 且不改变任何轮组字段。成功返回 SWERVE_OK。
 * 本函数不限制转向加速度，不处理舵向机械限位，不发送电流指令。
 */
Swerve_Status_t Swerve_Calculate(Swerve_Module_t *modules,
                                 size_t module_count,
                                 float vx_mps,
                                 float vy_mps,
                                 float wz_rad_s,
                                 float max_wheel_speed_mps);

/**
 * 保留现有函数名：轮子自身 RPM -> 轮缘线速度 m/s，保留转速正负号。
 * 输入不得为未经传动比换算的电机转子 RPM。空指针、非正半径、非有限输入
 * 或换算溢出返回 0；不会修改 module。
 */
float RPM_to_Thraed_speed(float rpm, const Swerve_Module_t *module);

#endif /* CHASSIS_SWERVE_H */
