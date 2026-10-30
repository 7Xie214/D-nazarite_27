/** @brief 四舵轮逆运动学实现，接口及坐标约定见 Swerve.h。 */
#include "Swerve.h"

#include <float.h>
#include <math.h>

#define SWERVE_PI       3.14159265358979323846f
#define SWERVE_TWO_PI   (2.0f * SWERVE_PI)
#define SWERVE_HALF_PI  (0.5f * SWERVE_PI)
#define SWERVE_ANGLE_ROUNDOFF_RAD (4.0f * FLT_EPSILON)

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

static int select_limited_angle(const Swerve_Module_t *module,
                                 float heading,
                                 float magnitude,
                                 float *angle,
                                 float *speed)
{
    float minimum, maximum;
    float best_distance = 0.0f;
    int found = 0;
    int k;

    if (module->angle_range == SWERVE_ANGLE_MINUS_90_TO_90) {
        minimum = -SWERVE_HALF_PI;
        maximum = SWERVE_HALF_PI;
    } else if (module->angle_range == SWERVE_ANGLE_0_TO_180) {
        minimum = 0.0f;
        maximum = SWERVE_PI;
    } else {
        return 0;
    }
    if ((module->actual_angle_rad < minimum) || (module->actual_angle_rad > maximum) ||
        (module->target_angle_rad < minimum) || (module->target_angle_rad > maximum)) {
        return 0;
    }
    if (magnitude <= SWERVE_SPEED_EPS_MPS) {
        *angle = module->target_angle_rad;
        *speed = 0.0f;
        return 1;
    }
    /* atan2 的范围是 [-pi, pi]，这五个候选覆盖两种半圈机械范围。 */
    for (k = -2; k <= 2; k++) {
        float candidate = heading + (float)k * SWERVE_PI;
        const float candidate_speed = ((k % 2) != 0) ? -magnitude : magnitude;
        float distance;
        /* atan2/加 pi 的舍入可能让端点偏离几个 ULP，仅消除这一数值误差。
         * 实际反馈与上次目标仍严格验界，不会在此归一化或钳位反馈。 */
        if (fabsf(candidate - minimum) <= SWERVE_ANGLE_ROUNDOFF_RAD) {
            candidate = minimum;
        } else if (fabsf(candidate - maximum) <= SWERVE_ANGLE_ROUNDOFF_RAD) {
            candidate = maximum;
        }
        if ((candidate < minimum) || (candidate > maximum)) {
            continue;
        }
        distance = fabsf(candidate - module->actual_angle_rad);
        if ((found == 0) || (distance < best_distance) ||
            ((distance == best_distance) && (candidate_speed > 0.0f) && (*speed < 0.0f))) {
            *angle = candidate;
            *speed = candidate_speed;
            best_distance = distance;
            found = 1;
        }
    }
    return found;
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

    *speed = hypotf(wheel_vx, wheel_vy);
    //小数点判断
    /* 非有限几何尺寸或速度会传播到轮速，统一在这里检查。 */
    if ((module->half_wheel_base < 0.0f) || (module->half_track_width < 0.0f) ||
        !isfinite(*speed) || !isfinite(module->actual_angle_rad) ||
        !isfinite(module->target_angle_rad)) {
        return 0;
    }
    if (module->angle_range != SWERVE_ANGLE_CONTINUOUS) {
        /* 先满足机械限位；不能在此之后再套用连续模式的最短角优化。 */
        return select_limited_angle(module, atan2f(wheel_vy, wheel_vx), *speed,
                                    angle, speed);
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
        if (!calculate_module(&modules[i], i, vx_mps, vy_mps, wz_rad_s,
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
