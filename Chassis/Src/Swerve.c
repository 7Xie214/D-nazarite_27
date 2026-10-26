/** @brief 四舵轮逆运动学实现，接口及坐标约定见 Swerve.h。 */
#include "Swerve.h"

#include <math.h>

#define SWERVE_PI       3.14159265358979323846f
#define SWERVE_TWO_PI   (2.0f * SWERVE_PI)
#define SWERVE_HALF_PI  (0.5f * SWERVE_PI)

/* 有界时间归一化，避免跨圈反馈导致循环减 2*pi。调用前确保 angle 有限。 */
static float wrap_angle(float angle)
{
    float wrapped = fmodf(angle, SWERVE_TWO_PI);
    if (wrapped >= SWERVE_PI) {
        wrapped -= SWERVE_TWO_PI;
    } else if (wrapped < -SWERVE_PI) {
        wrapped += SWERVE_TWO_PI;
    }
    return wrapped;
}

static int module_is_valid(const Swerve_Module_t *module)
{
    return isfinite(module->half_wheel_base) &&
           isfinite(module->half_track_width) &&
           (module->half_wheel_base >= 0.0f) &&
           (module->half_track_width >= 0.0f) &&
           isfinite(module->actual_angle_rad) &&
           isfinite(module->target_angle_rad);
}

static int calculate_module(const Swerve_Module_t *module,
                            size_t index,
                            float vx,
                            float vy,
                            float wz,
                            float *angle,
                            float *speed)
{
    /* 左前(+,+)、右前(+,-)、左后(-,+)、右后(-,-)。 */
    const float x = (index < 2u) ? module->half_wheel_base : -module->half_wheel_base;
    const float y = ((index == 0u) || (index == 2u)) ?
                    module->half_track_width : -module->half_track_width;
    const float wheel_vx = vx - wz * y;
    const float wheel_vy = vy + wz * x;
    float delta;

    if (!isfinite(wheel_vx) || !isfinite(wheel_vy)) {
        return 0;
    }
    *speed = hypotf(wheel_vx, wheel_vy);
    if (!isfinite(*speed)) {
        return 0;
    }
    if (*speed <= SWERVE_SPEED_EPS_MPS) {
        *angle = module->target_angle_rad;
        *speed = 0.0f;
        return 1;
    }

    /* 先缩小实际角再相减，避免大量累计圈数吞掉 atan2 给出的角度精度。 */
    delta = wrap_angle(atan2f(wheel_vy, wheel_vx) - wrap_angle(module->actual_angle_rad));
    if (fabsf(delta) > SWERVE_HALF_PI) {
        delta = wrap_angle(delta + SWERVE_PI);
        *speed = -*speed;
    }
    *angle = module->actual_angle_rad + delta;
    return isfinite(*angle);
}

Swerve_Status_t Swerve_Calculate(Swerve_Module_t *modules,
                                 size_t module_count,
                                 float vx_mps,
                                 float vy_mps,
                                 float wz_rad_s,
                                 float max_wheel_speed_mps)
{
    float angles[SWERVE_MODULE_COUNT];
    float speeds[SWERVE_MODULE_COUNT];
    float peak_speed = 0.0f;
    float scale = 1.0f;
    size_t i;

    if ((modules == NULL) || (module_count != SWERVE_MODULE_COUNT) ||
        !isfinite(vx_mps) || !isfinite(vy_mps) || !isfinite(wz_rad_s) ||
        !isfinite(max_wheel_speed_mps) || (max_wheel_speed_mps <= 0.0f)) {
        return SWERVE_ERR_PARAM;
    }

    /* 所有计算先写暂存区：任何一个轮组出错都不会留下半更新的整车目标。 */
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        if (!module_is_valid(&modules[i]) ||
            !calculate_module(&modules[i], i, vx_mps, vy_mps, wz_rad_s,
                               &angles[i], &speeds[i])) {
            return SWERVE_ERR_PARAM;
        }
        if (fabsf(speeds[i]) > peak_speed) {
            peak_speed = fabsf(speeds[i]);
        }
    }

    /* 共用一个缩放系数，保持平移与自转的比例；独立截断会改变运动方向。 */
    if (peak_speed > max_wheel_speed_mps) {
        scale = max_wheel_speed_mps / peak_speed;
    }
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        modules[i].V_x = vx_mps;
        modules[i].V_y = vy_mps;
        modules[i].W_z = wz_rad_s;
        modules[i].target_angle_rad = angles[i];
        modules[i].target_speed_mps = speeds[i] * scale;
    }
    return SWERVE_OK;
}

float RPM_to_Thraed_speed(float rpm, const Swerve_Module_t *module)
{
    float speed;
    if ((module == NULL) || !isfinite(rpm) ||
        !isfinite(module->steering_gear.R) || (module->steering_gear.R <= 0.0f)) {
        return 0.0f;
    }
    speed = (rpm / 60.0f) * SWERVE_TWO_PI * module->steering_gear.R;
    return isfinite(speed) ? speed : 0.0f;
}
