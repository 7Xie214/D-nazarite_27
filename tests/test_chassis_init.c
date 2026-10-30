#include "Chassis.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned int checks;
static unsigned int failures;

#define CHECK(condition) do { \
    checks++; \
    if (!(condition)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
    } \
} while (0)

#if CHASSIS_ENABLE_SWERVE
static void check_module(size_t index, uint16_t zero, int8_t steer_encoder,
                         int8_t steer_electric, int8_t drive_encoder,
                         int8_t drive_electric, Swerve_AngleRange_t range,
                         float initial_angle)
{
    const Swerve_Module_t *module = &g_swerve_modules[index];
    CHECK(module->rudder_direction.zero_offset == zero);
    CHECK(module->rudder_direction.encoder_direction == steer_encoder);
    CHECK(module->rudder_direction.electirc_direction == steer_electric);
    CHECK(module->steering_gear.encoder_direction == drive_encoder);
    CHECK(module->steering_gear.electric_direction == drive_electric);
    CHECK(module->angle_range == range);
    CHECK(module->actual_angle_rad == initial_angle);
    CHECK(module->target_angle_rad == initial_angle);
    CHECK(isfinite(module->steering_gear.R) &&
          fabsf(module->steering_gear.R - CHASSIS_SWERVE_WHEEL_RADIUS_M) < 1.0e-6f);
    CHECK(isfinite(module->half_wheel_base) &&
          fabsf(module->half_wheel_base - CHASSIS_SWERVE_HALF_WHEEL_BASE_M) < 1.0e-6f);
    CHECK(isfinite(module->half_track_width) &&
          fabsf(module->half_track_width - CHASSIS_SWERVE_HALF_TRACK_WIDTH_M) < 1.0e-6f);
    CHECK(module->V_x == 0.0f);
    CHECK(module->V_y == 0.0f);
    CHECK(module->W_z == 0.0f);
    CHECK(module->target_speed_mps == 0.0f);
}

static void check_configuration(void)
{
    check_module(SWERVE_FRONT_LEFT,
                  CHASSIS_SWERVE_FL_ZERO_OFFSET,
                  CHASSIS_SWERVE_FL_STEER_ENCODER_DIRECTION,
                  CHASSIS_SWERVE_FL_STEER_ELECTRIC_DIRECTION,
                  CHASSIS_SWERVE_FL_DRIVE_ENCODER_DIRECTION,
                  CHASSIS_SWERVE_FL_DRIVE_ELECTRIC_DIRECTION,
                  CHASSIS_SWERVE_FL_ANGLE_RANGE,
                  CHASSIS_SWERVE_FL_INITIAL_ANGLE_RAD);
    check_module(SWERVE_FRONT_RIGHT,
                  CHASSIS_SWERVE_FR_ZERO_OFFSET,
                  CHASSIS_SWERVE_FR_STEER_ENCODER_DIRECTION,
                  CHASSIS_SWERVE_FR_STEER_ELECTRIC_DIRECTION,
                  CHASSIS_SWERVE_FR_DRIVE_ENCODER_DIRECTION,
                  CHASSIS_SWERVE_FR_DRIVE_ELECTRIC_DIRECTION,
                  CHASSIS_SWERVE_FR_ANGLE_RANGE,
                  CHASSIS_SWERVE_FR_INITIAL_ANGLE_RAD);
    check_module(SWERVE_REAR_LEFT,
                  CHASSIS_SWERVE_RL_ZERO_OFFSET,
                  CHASSIS_SWERVE_RL_STEER_ENCODER_DIRECTION,
                  CHASSIS_SWERVE_RL_STEER_ELECTRIC_DIRECTION,
                  CHASSIS_SWERVE_RL_DRIVE_ENCODER_DIRECTION,
                  CHASSIS_SWERVE_RL_DRIVE_ELECTRIC_DIRECTION,
                  CHASSIS_SWERVE_RL_ANGLE_RANGE,
                  CHASSIS_SWERVE_RL_INITIAL_ANGLE_RAD);
    check_module(SWERVE_REAR_RIGHT,
                  CHASSIS_SWERVE_RR_ZERO_OFFSET,
                  CHASSIS_SWERVE_RR_STEER_ENCODER_DIRECTION,
                  CHASSIS_SWERVE_RR_STEER_ELECTRIC_DIRECTION,
                  CHASSIS_SWERVE_RR_DRIVE_ENCODER_DIRECTION,
                  CHASSIS_SWERVE_RR_DRIVE_ELECTRIC_DIRECTION,
                  CHASSIS_SWERVE_RR_ANGLE_RANGE,
                  CHASSIS_SWERVE_RR_INITIAL_ANGLE_RAD);
}
#endif

int main(void)
{
#if CHASSIS_ENABLE_SWERVE
    memset(g_swerve_modules, 0xA5, sizeof(g_swerve_modules));
    Chassis_Init();
    check_configuration();
    /* Each wheel must own independent mutable state. */
    g_swerve_modules[0].rudder_direction.zero_offset ^= 1u;
    CHECK(g_swerve_modules[1].rudder_direction.zero_offset == CHASSIS_SWERVE_FR_ZERO_OFFSET);
    CHECK(g_swerve_modules[2].rudder_direction.zero_offset == CHASSIS_SWERVE_RL_ZERO_OFFSET);
    CHECK(g_swerve_modules[3].rudder_direction.zero_offset == CHASSIS_SWERVE_RR_ZERO_OFFSET);
    g_swerve_modules[3].actual_angle_rad = 123.0f;
    g_swerve_modules[2].target_speed_mps = -12.0f;
    g_swerve_modules[1].V_x = 10.0f;
    Chassis_Init();
    check_configuration();
#else
    /* Reserved/disabled branches must link without introducing wheel types. */
    Chassis_Init();
    Chassis_Init();
    CHECK(1);
#endif
    printf("%u chassis-init checks, %u failures\n", checks, failures);
    return (failures == 0u) ? 0 : 1;
}
