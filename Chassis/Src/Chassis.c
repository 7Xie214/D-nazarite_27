/** @brief 按编译期底盘选择加载软件配置。 */
#include "Chassis.h"

#if CHASSIS_ENABLE_SWERVE

Swerve_Module_t g_swerve_modules[SWERVE_MODULE_COUNT];

/* 一个配置块只描述一个轮组，方向系数仅保存，不在初始化时参与解算。
 * STEER_ELECTRIC_DIRECTION 对接现有字段 electirc_direction，保留其拼写。 */
#define SWERVE_MODULE_INITIALIZER(module_prefix) { \
    .rudder_direction = { \
        .zero_offset = module_prefix##_ZERO_OFFSET, \
        .encoder_direction = module_prefix##_STEER_ENCODER_DIRECTION, \
        .electirc_direction = module_prefix##_STEER_ELECTRIC_DIRECTION \
    }, \
    .steering_gear = { \
        .electric_direction = module_prefix##_DRIVE_ELECTRIC_DIRECTION, \
        .encoder_direction = module_prefix##_DRIVE_ENCODER_DIRECTION, \
        .R = CHASSIS_SWERVE_WHEEL_RADIUS_M \
    }, \
    .V_x = 0.0f, \
    .V_y = 0.0f, \
    .W_z = 0.0f, \
    .target_angle_rad = module_prefix##_INITIAL_ANGLE_RAD, \
    .target_speed_mps = 0.0f, \
    .half_wheel_base = CHASSIS_SWERVE_HALF_WHEEL_BASE_M, \
    .half_track_width = CHASSIS_SWERVE_HALF_TRACK_WIDTH_M, \
    .actual_angle_rad = module_prefix##_INITIAL_ANGLE_RAD, \
    .angle_range = module_prefix##_ANGLE_RANGE \
}

static const Swerve_Module_t s_swerve_configuration[SWERVE_MODULE_COUNT] = {
    [SWERVE_FRONT_LEFT] = SWERVE_MODULE_INITIALIZER(CHASSIS_SWERVE_FL),
    [SWERVE_FRONT_RIGHT] = SWERVE_MODULE_INITIALIZER(CHASSIS_SWERVE_FR),
    [SWERVE_REAR_LEFT] = SWERVE_MODULE_INITIALIZER(CHASSIS_SWERVE_RL),
    [SWERVE_REAR_RIGHT] = SWERVE_MODULE_INITIALIZER(CHASSIS_SWERVE_RR)
};

#undef SWERVE_MODULE_INITIALIZER
#endif /* CHASSIS_ENABLE_SWERVE */

void Chassis_Init(void)
{
#if CHASSIS_ENABLE_MECANUM
    /* 预留：麦轮的软件初始化。 */
#elif CHASSIS_ENABLE_OMNIWHEEL
    /* 预留：全向轮的软件初始化。 */
#elif CHASSIS_ENABLE_SWERVE
    size_t i;
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        g_swerve_modules[i] = s_swerve_configuration[i];
    }
#endif
}
