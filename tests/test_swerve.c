#include "Swerve.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_EPS 2.0e-5f

static unsigned int failures;
static unsigned int checks;

#define CHECK(condition) do { \
    checks++; \
    if (!(condition)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
    } \
} while (0)

#define NEAR(actual, expected) do { \
    const float test_actual = (actual); \
    const float test_expected = (expected); \
    if (!isfinite(test_actual) || fabsf(test_actual - test_expected) > TEST_EPS) { \
        printf("actual=%.9g expected=%.9g\n", (double)test_actual, (double)test_expected); \
    } \
    CHECK(isfinite(test_actual) && \
          fabsf(test_actual - test_expected) <= TEST_EPS); \
} while (0)

static void setup(Swerve_Module_t modules[SWERVE_MODULE_COUNT])
{
    size_t i;
    memset(modules, 0, sizeof(Swerve_Module_t) * SWERVE_MODULE_COUNT);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        modules[i].half_wheel_base = 0.3f;
        modules[i].half_track_width = 0.2f;
        modules[i].steering_gear.R = 0.1f;
        modules[i].rudder_direction.encoder_direction = 1;
        modules[i].rudder_direction.electirc_direction = 1;
        modules[i].steering_gear.encoder_direction = 1;
        modules[i].steering_gear.electric_direction = 1;
        modules[i].angle_range = SWERVE_ANGLE_CONTINUOUS;
    }
}

static void test_basic_motion(void)
{
    static const float commands[][3] = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
        {0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f},
        {0.8f, -0.4f, 1.7f}
    };
    static const float positions[4][2] = {
        {0.3f, 0.2f}, {0.3f, -0.2f}, {-0.3f, 0.2f}, {-0.3f, -0.2f}
    };
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    size_t c, i;
    for (c = 0u; c < sizeof(commands) / sizeof(commands[0]); c++) {
        setup(modules);
        CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT,
                               commands[c][0], commands[c][1],
                               commands[c][2], 10.0f) == SWERVE_OK);
        for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
            const float angle = modules[i].target_angle_rad;
            const float speed = modules[i].target_speed_mps;
            NEAR(speed * cosf(angle), commands[c][0] - commands[c][2] * positions[i][1]);
            NEAR(speed * sinf(angle), commands[c][1] + commands[c][2] * positions[i][0]);
            CHECK(fabsf(angle - modules[i].actual_angle_rad) <= TEST_PI / 2.0f + TEST_EPS);
            NEAR(modules[i].V_x, commands[c][0]);
            NEAR(modules[i].V_y, commands[c][1]);
            NEAR(modules[i].W_z, commands[c][2]);
        }
    }
    /* Explicit reversal: no steering movement is needed to drive backwards. */
    setup(modules);
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT, -1.0f, 0.0f, 0.0f, 2.0f) == SWERVE_OK);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        NEAR(modules[i].target_angle_rad, 0.0f);
        NEAR(modules[i].target_speed_mps, -1.0f);
    }
}

static void test_wrap_and_multiturn(void)
{
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    const float desired = -179.0f * TEST_PI / 180.0f;
    size_t i;
    setup(modules);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        modules[i].actual_angle_rad = 179.0f * TEST_PI / 180.0f;
    }
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT,
                           cosf(desired), sinf(desired), 0.0f, 2.0f) == SWERVE_OK);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        NEAR(modules[i].target_angle_rad, 181.0f * TEST_PI / 180.0f);
        NEAR(modules[i].target_speed_mps, 1.0f);
        modules[i].actual_angle_rad = 4.0f * TEST_PI + TEST_PI / 4.0f;
    }
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT, 1.0f, 1.0f, 0.0f, 2.0f) == SWERVE_OK);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        NEAR(modules[i].target_angle_rad, modules[i].actual_angle_rad);
        NEAR(modules[i].target_speed_mps, sqrtf(2.0f));
    }
}

static void test_stationary_hold(void)
{
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    size_t i;
    setup(modules);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        modules[i].target_angle_rad = 0.4f + (float)i;
    }
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT, 0.0f, 0.0f, 0.0f, 2.0f) == SWERVE_OK);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        NEAR(modules[i].target_angle_rad, 0.4f + (float)i);
        NEAR(modules[i].target_speed_mps, 0.0f);
    }
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT,
                           SWERVE_SPEED_EPS_MPS / 2.0f, 0.0f, 0.0f, 2.0f) == SWERVE_OK);
    NEAR(modules[0].target_angle_rad, 0.4f);
    NEAR(modules[0].target_speed_mps, 0.0f);
    /* Body command is nonzero but the front-left wheel is the rotation center. */
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT, 0.2f, -0.3f, 1.0f, 2.0f) == SWERVE_OK);
    NEAR(modules[0].target_angle_rad, 0.4f);
    NEAR(modules[0].target_speed_mps, 0.0f);
    CHECK(fabsf(modules[1].target_speed_mps) > 0.0f);
}

static void test_heading_boundaries(void)
{
    static const float desired_degrees[] = {90.0f, 91.0f, -90.0f, -91.0f, 180.0f, -180.0f};
    static const float actual_angles[] = {0.0f, 3.12f, -3.12f, -4.0f * TEST_PI - 0.2f};
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    size_t d, a, i;
    for (a = 0u; a < sizeof(actual_angles) / sizeof(actual_angles[0]); a++) {
        for (d = 0u; d < sizeof(desired_degrees) / sizeof(desired_degrees[0]); d++) {
            const float desired = desired_degrees[d] * TEST_PI / 180.0f;
            setup(modules);
            for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
                modules[i].actual_angle_rad = actual_angles[a];
            }
            CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT,
                                   cosf(desired), sinf(desired), 0.0f, 2.0f) == SWERVE_OK);
            for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
                const float target = modules[i].target_angle_rad;
                const float speed = modules[i].target_speed_mps;
                CHECK(fabsf(target - actual_angles[a]) <= TEST_PI / 2.0f + TEST_EPS);
                NEAR(speed * cosf(target), cosf(desired));
                NEAR(speed * sinf(target), sinf(desired));
            }
        }
    }
}

static void test_common_speed_limit(void)
{
    Swerve_Module_t free_modules[SWERVE_MODULE_COUNT];
    Swerve_Module_t limited[SWERVE_MODULE_COUNT];
    float peak = 0.0f;
    size_t i;
    setup(free_modules);
    setup(limited);
    CHECK(Swerve_Calculate(free_modules, SWERVE_MODULE_COUNT, 1.0f, 0.5f, 2.0f, 10.0f) == SWERVE_OK);
    CHECK(Swerve_Calculate(limited, SWERVE_MODULE_COUNT, 1.0f, 0.5f, 2.0f, 0.6f) == SWERVE_OK);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        if (fabsf(free_modules[i].target_speed_mps) > peak) {
            peak = fabsf(free_modules[i].target_speed_mps);
        }
    }
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        NEAR(limited[i].target_speed_mps, free_modules[i].target_speed_mps * (0.6f / peak));
        NEAR(limited[i].target_angle_rad, free_modules[i].target_angle_rad);
        CHECK(fabsf(limited[i].target_speed_mps) <= 0.6f + TEST_EPS);
    }
}

static void test_calibration_is_external(void)
{
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    size_t i;
    setup(modules);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        modules[i].rudder_direction.encoder_direction = -1;
        modules[i].rudder_direction.electirc_direction = -1;
        modules[i].rudder_direction.zero_offset = 7000u;
        modules[i].steering_gear.encoder_direction = -1;
        modules[i].steering_gear.electric_direction = -1;
        modules[i].steering_gear.R = 0.0f;
    }
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f) == SWERVE_OK);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        NEAR(modules[i].target_speed_mps, 1.0f);
        NEAR(modules[i].target_angle_rad, 0.0f);
        CHECK(modules[i].rudder_direction.zero_offset == 7000u);
    }
}

static void expect_rejected(Swerve_Module_t modules[SWERVE_MODULE_COUNT],
                            size_t count, float vx, float vy, float wz, float limit)
{
    unsigned char before[sizeof(Swerve_Module_t) * SWERVE_MODULE_COUNT];
    memcpy(before, modules, sizeof(before));
    CHECK(Swerve_Calculate(modules, count, vx, vy, wz, limit) == SWERVE_ERR_PARAM);
    CHECK(memcmp(before, modules, sizeof(before)) == 0);
}

static void test_invalid_inputs(void)
{
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    setup(modules);
    CHECK(Swerve_Calculate(NULL, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f) == SWERVE_ERR_PARAM);
    expect_rejected(modules, 0u, 1.0f, 0.0f, 0.0f, 2.0f);
    expect_rejected(modules, 3u, 1.0f, 0.0f, 0.0f, 2.0f);
    expect_rejected(modules, 5u, 1.0f, 0.0f, 0.0f, 2.0f);
    expect_rejected(modules, SWERVE_MODULE_COUNT, NAN, 0.0f, 0.0f, 2.0f);
    expect_rejected(modules, SWERVE_MODULE_COUNT, 0.0f, INFINITY, 0.0f, 2.0f);
    expect_rejected(modules, SWERVE_MODULE_COUNT, 0.0f, 0.0f, NAN, 2.0f);
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 0.0f);
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, -1.0f);
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, INFINITY);
    modules[3].half_wheel_base = -0.3f;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f);
    setup(modules);
    modules[3].half_track_width = NAN;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f);
    setup(modules);
    modules[3].actual_angle_rad = INFINITY;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f);
    setup(modules);
    modules[3].target_angle_rad = NAN;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f);
    setup(modules);
    modules[3].half_wheel_base = FLT_MAX;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, FLT_MAX, 2.0f);
}

static void test_rpm_conversion(void)
{
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    setup(modules);
    NEAR(RPM_to_Thraed_speed(60.0f, &modules[0]), 0.2f * TEST_PI);
    NEAR(RPM_to_Thraed_speed(-60.0f, &modules[0]), -0.2f * TEST_PI);
    NEAR(RPM_to_Thraed_speed(0.0f, &modules[0]), 0.0f);
    NEAR(RPM_to_Thraed_speed(60.0f, NULL), 0.0f);
    NEAR(RPM_to_Thraed_speed(NAN, &modules[0]), 0.0f);
    modules[0].steering_gear.R = 0.0f;
    NEAR(RPM_to_Thraed_speed(60.0f, &modules[0]), 0.0f);
    modules[0].steering_gear.R = -0.1f;
    NEAR(RPM_to_Thraed_speed(60.0f, &modules[0]), 0.0f);
    modules[0].steering_gear.R = INFINITY;
    NEAR(RPM_to_Thraed_speed(60.0f, &modules[0]), 0.0f);
    modules[0].steering_gear.R = FLT_MAX;
    NEAR(RPM_to_Thraed_speed(FLT_MAX, &modules[0]), 0.0f);
}

static void test_default_angle_range(void)
{
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    const float desired = 100.0f * TEST_PI / 180.0f;
    size_t i;
    setup(modules);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        modules[i].angle_range = (Swerve_AngleRange_t)0;
        modules[i].actual_angle_rad = 80.0f * TEST_PI / 180.0f;
        modules[i].target_angle_rad = modules[i].actual_angle_rad;
    }
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT,
                           cosf(desired), sinf(desired), 0.0f, 2.0f) == SWERVE_OK);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        CHECK(modules[i].target_angle_rad >= -TEST_PI / 2.0f);
        CHECK(modules[i].target_angle_rad <= TEST_PI / 2.0f);
        NEAR(modules[i].target_angle_rad, -80.0f * TEST_PI / 180.0f);
        NEAR(modules[i].target_speed_mps, -1.0f);
    }
}

static void setup_limited(Swerve_Module_t modules[SWERVE_MODULE_COUNT],
                          Swerve_AngleRange_t mode, float actual_degrees)
{
    size_t i;
    setup(modules);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        modules[i].angle_range = mode;
        modules[i].actual_angle_rad = actual_degrees * TEST_PI / 180.0f;
        modules[i].target_angle_rad = modules[i].actual_angle_rad;
    }
}

static void test_limited_heading_vectors(void)
{
    static const float directions[] = {
        -180.0f, -150.0f, -100.0f, -90.01f, -90.0f, -89.99f, -60.0f, -10.0f,
        0.0f, 10.0f, 60.0f, 89.99f, 90.0f, 90.01f, 100.0f, 150.0f,
        179.99f, 180.0f, 180.01f
    };
    static const Swerve_AngleRange_t modes[] = {
        SWERVE_ANGLE_MINUS_90_TO_90, SWERVE_ANGLE_0_TO_180
    };
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    size_t m, a, d, i;
    for (m = 0u; m < sizeof(modes) / sizeof(modes[0]); m++) {
        const float low_degrees = (modes[m] == SWERVE_ANGLE_0_TO_180) ? 0.0f : -90.0f;
        const float low = (modes[m] == SWERVE_ANGLE_0_TO_180) ? 0.0f : -TEST_PI / 2.0f;
        const float high = (modes[m] == SWERVE_ANGLE_0_TO_180) ? TEST_PI : TEST_PI / 2.0f;
        for (a = 0u; a < 3u; a++) {
            const float actual_degrees = low_degrees + (float)a * 90.0f;
            for (d = 0u; d < sizeof(directions) / sizeof(directions[0]); d++) {
                const float desired = directions[d] * TEST_PI / 180.0f;
                setup_limited(modules, modes[m], actual_degrees);
                CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT,
                                       cosf(desired), sinf(desired), 0.0f, 0.4f) == SWERVE_OK);
                for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
                    const float angle = modules[i].target_angle_rad;
                    const float speed = modules[i].target_speed_mps;
                    if ((i == 0u) && ((angle < low) || (angle > high))) {
                        printf("mode=%d actual_deg=%g desired_deg=%g angle=%.9g bounds=[%.9g,%.9g]\n",
                               (int)modes[m], (double)actual_degrees, (double)directions[d],
                               (double)angle, (double)low, (double)high);
                    }
                    CHECK(angle >= low);
                    CHECK(angle <= high);
                    NEAR(speed * cosf(angle), 0.4f * cosf(desired));
                    NEAR(speed * sinf(angle), 0.4f * sinf(desired));
                }
            }
        }
    }
}

static void test_nearest_legal_heading(void)
{
    static const struct {
        Swerve_AngleRange_t mode;
        float actual_degrees, desired_degrees, target_degrees, speed;
    } cases[] = {
        {SWERVE_ANGLE_0_TO_180, 170.0f, -10.0f, 170.0f, -1.0f},
        {SWERVE_ANGLE_0_TO_180, 170.0f, 0.0f, 180.0f, -1.0f},
        {SWERVE_ANGLE_0_TO_180, 10.0f, 180.0f, 0.0f, -1.0f},
        /* 190 degrees would be shorter but is outside the mechanical range. */
        {SWERVE_ANGLE_0_TO_180, 170.0f, 10.0f, 10.0f, 1.0f},
        {SWERVE_ANGLE_MINUS_90_TO_90, 0.0f, 120.0f, -60.0f, -1.0f},
        {SWERVE_ANGLE_MINUS_90_TO_90, 80.0f, -90.0f, 90.0f, -1.0f},
        {SWERVE_ANGLE_MINUS_90_TO_90, 0.0f, 90.0f, 90.0f, 1.0f},
        {SWERVE_ANGLE_MINUS_90_TO_90, 80.0f, 90.01f, -89.99f, -1.0f},
        {SWERVE_ANGLE_MINUS_90_TO_90, -80.0f, -90.01f, 89.99f, -1.0f},
        {SWERVE_ANGLE_0_TO_180, 170.0f, 180.01f, 0.01f, -1.0f}
    };
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    size_t c, i;
    for (c = 0u; c < sizeof(cases) / sizeof(cases[0]); c++) {
        const float desired = cases[c].desired_degrees * TEST_PI / 180.0f;
        setup_limited(modules, cases[c].mode, cases[c].actual_degrees);
        CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT,
                               cosf(desired), sinf(desired), 0.0f, 2.0f) == SWERVE_OK);
        if (fabsf(modules[0].target_speed_mps - cases[c].speed) > TEST_EPS) {
            printf("nearest-heading case=%u mode=%d actual_deg=%g desired_deg=%g\n",
                   (unsigned int)c, (int)cases[c].mode,
                   (double)cases[c].actual_degrees, (double)cases[c].desired_degrees);
        }
        for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
            NEAR(modules[i].target_angle_rad, cases[c].target_degrees * TEST_PI / 180.0f);
            NEAR(modules[i].target_speed_mps, cases[c].speed);
        }
    }
}

static void test_limited_hold_and_errors(void)
{
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    size_t i;
    setup_limited(modules, SWERVE_ANGLE_0_TO_180, 45.0f);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        modules[i].target_angle_rad = TEST_PI / 3.0f;
    }
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT, 0.0f, 0.0f, 0.0f, 2.0f) == SWERVE_OK);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        NEAR(modules[i].target_angle_rad, TEST_PI / 3.0f);
        NEAR(modules[i].target_speed_mps, 0.0f);
    }
    modules[3].angle_range = (Swerve_AngleRange_t)99;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f);
    setup_limited(modules, SWERVE_ANGLE_0_TO_180, 45.0f);
    modules[3].actual_angle_rad = -0.01f;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f);
    modules[3].actual_angle_rad = TEST_PI + 0.01f;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 0.0f, 0.0f, 0.0f, 2.0f);
    setup_limited(modules, SWERVE_ANGLE_MINUS_90_TO_90, 45.0f);
    modules[3].target_angle_rad = TEST_PI;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f);
    expect_rejected(modules, SWERVE_MODULE_COUNT, 0.0f, 0.0f, 0.0f, 2.0f);
    setup_limited(modules, SWERVE_ANGLE_MINUS_90_TO_90, 45.0f);
    modules[3].actual_angle_rad = -TEST_PI;
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f);
    setup_limited(modules, SWERVE_ANGLE_MINUS_90_TO_90, 45.0f);
    modules[3].actual_angle_rad = nextafterf(TEST_PI / 2.0f, INFINITY);
    expect_rejected(modules, SWERVE_MODULE_COUNT, 1.0f, 0.0f, 0.0f, 2.0f);
    setup_limited(modules, SWERVE_ANGLE_MINUS_90_TO_90, 45.0f);
    modules[3].target_angle_rad = nextafterf(-TEST_PI / 2.0f, -INFINITY);
    expect_rejected(modules, SWERVE_MODULE_COUNT, 0.0f, 0.0f, 0.0f, 2.0f);
}

static void test_mixed_angle_ranges(void)
{
    Swerve_Module_t modules[SWERVE_MODULE_COUNT];
    const float desired = 100.0f * TEST_PI / 180.0f;
    setup(modules);
    modules[0].angle_range = SWERVE_ANGLE_MINUS_90_TO_90;
    modules[0].actual_angle_rad = 80.0f * TEST_PI / 180.0f;
    modules[1].angle_range = SWERVE_ANGLE_0_TO_180;
    modules[1].actual_angle_rad = 170.0f * TEST_PI / 180.0f;
    CHECK(Swerve_Calculate(modules, SWERVE_MODULE_COUNT,
                           cosf(desired), sinf(desired), 0.0f, 2.0f) == SWERVE_OK);
    NEAR(modules[0].target_angle_rad, -80.0f * TEST_PI / 180.0f);
    NEAR(modules[0].target_speed_mps, -1.0f);
    NEAR(modules[1].target_angle_rad, 100.0f * TEST_PI / 180.0f);
    NEAR(modules[1].target_speed_mps, 1.0f);
}

int main(void)
{
    test_basic_motion();
    test_wrap_and_multiturn();
    test_stationary_hold();
    test_heading_boundaries();
    test_common_speed_limit();
    test_calibration_is_external();
    test_invalid_inputs();
    test_rpm_conversion();
    test_default_angle_range();
    test_limited_heading_vectors();
    test_nearest_legal_heading();
    test_limited_hold_and_errors();
    test_mixed_angle_ranges();
    printf("%u checks, %u failures\n", checks, failures);
    return (failures == 0u) ? 0 : 1;
}
