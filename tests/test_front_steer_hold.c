/* Real task + APP + Device: three GM6020 zero holds, right rear released. */
#include "ChassisTask.h"
#include "ChassisApp.h"
#include "fdcan.h"
#include "My_CAN.h"
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
osThreadId_t Chassis_TaskHandle = (osThreadId_t)1;
osThreadId_t Upstairs_TaskHandle = (osThreadId_t)2;
uint32_t test_irq_mask;
static uint32_t tick;
static unsigned int checks, failures, frames, wrong_frames;
static int16_t packet[2][4];
static jmp_buf done;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("FAIL %d: %s\n", __LINE__, #c); } } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b)) < 0.0001f)

uint32_t HAL_GetTick(void) { return tick; }
uint32_t osKernelGetTickCount(void) { return tick; }
uint32_t osKernelGetTickFreq(void) { return 1000u; }
uint32_t osThreadFlagsWait(uint32_t f, uint32_t o, uint32_t t)
{ (void)f; (void)o; (void)t; return osFlagsErrorResource; }
uint32_t osThreadFlagsClear(uint32_t f) { (void)f; return 0u; }
uint32_t osThreadFlagsSet(osThreadId_t t, uint32_t f) { (void)t; return f; }
int32_t osKernelLock(void) { return 0; }
int32_t osKernelRestoreLock(int32_t s) { return s; }
void osThreadExit(void) { longjmp(done, 1); }
void Test_EncoderOnCanFrame(FDCAN_HandleTypeDef *h, uint16_t id, const uint8_t *d, uint8_t n)
{ (void)h; (void)id; (void)d; (void)n; }
HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *h, uint16_t id, const uint8_t *d, uint8_t n)
{
    unsigned int i, bus = h == &hfdcan1 ? 0u : 1u;
    if ((h != &hfdcan1 && h != &hfdcan2) || id != 0x1ffu || n != 8u) {
        wrong_frames++; return HAL_OK;
    }
    for (i = 0u; i < 4u; i++) {
        uint16_t v = (uint16_t)((uint16_t)d[2u*i] << 8u) | d[2u*i+1u];
        packet[bus][i] = (int16_t)v;
    }
    frames++;
    return HAL_OK;
}
static void feed(DjiMotor_t *m, uint16_t raw)
{
    uint8_t d[8] = {0u};
    d[0] = (uint8_t)(raw >> 8u); d[1] = (uint8_t)raw;
    DjiMotor_OnCanFrame(m->hfdcan, (uint16_t)(0x204u + m->id), d, 8u);
}
static void inspect(void)
{
    if (tick == 1u) {
        CHECK(g_motor_6020_1.cmd == 0 && g_motor_6020_4.cmd == 0);
        CHECK(g_chassis_wheels[0].feedback_ready == 0u);
        CHECK(g_motor_6020_1.angle_seeded == 0u);
    } else if (tick == 2u || tick == 3u) {
        CHECK(g_motor_6020_1.cmd < 0);
        CHECK(g_motor_6020_4.cmd > 0);
        CHECK(g_motor_6020_2.cmd < 0 && g_motor_6020_3.cmd == 0);
        NEAR(g_swerve_modules[0].actual_angle_rad, 0.0030679616f); /* +4 counts */
        NEAR(g_swerve_modules[1].actual_angle_rad, 0.0981747704f); /* +128 counts */
        NEAR(g_swerve_modules[0].target_angle_rad, 0.0f);
        NEAR(g_swerve_modules[1].target_angle_rad, 0.0f);
    } else if (tick == 4u) {
        CHECK(g_motor_6020_1.cmd == 0 && g_motor_6020_4.cmd == 0);
        CHECK(g_motor_6020_2.cmd == 0 && g_motor_6020_3.cmd == 0);
    } else if (tick == 55u) {
        CHECK(g_motor_6020_1.cmd == 0);
        CHECK(g_chassis_wheels[0].feedback_ready == 0u);
        CHECK(g_motor_6020_4.cmd > 0); /* Missing FL does not suppress FR. */
        CHECK(g_motor_6020_2.cmd == 0 && g_motor_6020_3.cmd == 0);
        CHECK(g_chassis_wheels[2].feedback_ready == 0u);
        NEAR(g_chassis_wheels[0].steer_pid.spd.integral, 0.0f);
    } else if (tick == 56u) {
        CHECK(g_motor_6020_1.cmd > 0); /* Recovered 4 counts below mechanical zero. */
        CHECK(g_chassis_wheels[0].feedback_ready != 0u);
        CHECK(g_motor_6020_2.cmd < 0 && g_chassis_wheels[2].feedback_ready != 0u);
        NEAR(g_swerve_modules[0].actual_angle_rad, -0.0030679616f);
    } else if (tick == 57u) {
        CHECK(g_motor_6020_1.cmd == 0); /* Outside configured +/-90 degree travel. */
        CHECK(g_motor_6020_4.cmd > 0);
        CHECK(g_motor_6020_2.cmd == 0 && g_motor_6020_3.cmd == 0);
    }
    CHECK(packet[0][0] == g_motor_6020_1.cmd);
    CHECK(packet[1][3] == g_motor_6020_4.cmd);
    CHECK(packet[0][1] == g_motor_6020_2.cmd && packet[0][2] == 0 && packet[0][3] == 0);
    CHECK(packet[1][2] == g_motor_6020_3.cmd && packet[1][0] == 0 && packet[1][1] == 0);
    CHECK(g_motor_6020_3.cmd == 0 && packet[1][2] == 0);
    CHECK(g_motor_3508_4.cmd == 0 && g_motor_3508_5.cmd == 0);
}
osStatus_t osDelayUntil(uint32_t deadline)
{
    if (tick == 0u) {
        /* Conversion/zero-heading fixture is pure P/P; user PI tuning varies. */
        unsigned int i;
        for (i = 0u; i < 4u; i++) {
            g_chassis_wheels[i].steer_pid.pos.Ki = 0.0f;
            g_chassis_wheels[i].steer_pid.spd.Ki = 0.0f;
        }
    }
    if (tick != 0u) { inspect(); }
    if (tick == 57u) { longjmp(done, 1); }
    CHECK(deadline - tick == 1u);
    tick = deadline;
    if (tick == 2u || tick == 3u) {
        feed(&g_motor_6020_1, 2u); feed(&g_motor_6020_4, 872u);
        feed(&g_motor_6020_2, 1623u); feed(&g_motor_6020_3, 6562u);
    } else if (tick == 4u) {
        feed(&g_motor_6020_1, 8190u); feed(&g_motor_6020_4, 1000u);
        feed(&g_motor_6020_2, 1495u); feed(&g_motor_6020_3, 6690u);
    } else if (tick > 4u) {
        feed(&g_motor_6020_4, 872u);
        feed(&g_motor_6020_3, 6562u);
        if (tick == 56u) { feed(&g_motor_6020_1, 8186u); }
        if (tick == 57u) { feed(&g_motor_6020_1, 3000u); }
        if (tick == 56u) { feed(&g_motor_6020_2, 1559u); }
        if (tick == 57u) { feed(&g_motor_6020_2, 4495u); }
    }
    return osOK;
}
int main(void)
{
    DjiMotor_Init(); Chassis_Init();
    g_swerve_modules[0].rudder_direction.zero_offset = 8190u;
    g_swerve_modules[1].rudder_direction.zero_offset = 1000u;
    g_swerve_modules[1].rudder_direction.encoder_direction = -1;
    g_swerve_modules[1].rudder_direction.electirc_direction = -1;
    g_swerve_modules[2].rudder_direction.zero_offset = 1495u;
    g_swerve_modules[3].rudder_direction.zero_offset = 6690u;
    if (setjmp(done) == 0) { ChassisTask(NULL); }
    CHECK(tick == 57u);
    CHECK(frames == 114u && wrong_frames == 0u);
    CHECK(ChassisTask_RequestFrontMove() == osErrorResource);
    CHECK(ChassisTask_PollReply() == 0u);

    /* A speed disturbance at mechanical zero still gets damping voltage. */
    feed(&g_motor_6020_1, 8190u); feed(&g_motor_6020_4, 1000u);
    g_motor_6020_1.raw_speed = 60;
    ChassisApp_UpdateFrontSteerHold(0.001f);
    CHECK(g_motor_6020_1.cmd < 0);
    CHECK(g_motor_6020_4.cmd == 0);
    /* The tuned voltage cap also limits braking under a large speed error. */
    g_motor_6020_1.raw_speed = 1000;
    ChassisApp_UpdateFrontSteerHold(0.001f);
    CHECK(g_motor_6020_1.cmd == -24000 && packet[0][0] == -24000);
    /* Saturate a deliberately excessive gain without wrapping the CAN command. */
    g_chassis_wheels[0].steer_pid.spd.Kp = 1.0e9f;
    g_chassis_wheels[0].steer_pid.spd.out_max = 1.0e9f;
    ChassisApp_UpdateFrontSteerHold(0.001f);
    CHECK(g_motor_6020_1.cmd == -DJI_MODEL_6020.cmd_max && packet[0][0] == -DJI_MODEL_6020.cmd_max);
    ChassisApp_UpdateFrontSteerHold(NAN);
    CHECK(g_motor_6020_1.cmd == 0 && g_motor_6020_4.cmd == 0);
    CHECK(g_chassis_app_status == CHASSIS_APP_FAULT_DT);

    /* [0,180] must approach zero from its valid side, without crossing bounds. */
    g_motor_6020_1.raw_speed = 0;
    g_swerve_modules[0].angle_range = SWERVE_ANGLE_0_TO_180;
    feed(&g_motor_6020_1, 4094u); /* +4096 counts from calibrated zero */
    ChassisApp_UpdateFrontSteerHold(0.001f);
    NEAR(g_swerve_modules[0].actual_angle_rad, 3.14159265359f);
    CHECK(g_motor_6020_1.cmd < 0);
    feed(&g_motor_6020_1, 4095u); /* one count outside the selected range */
    ChassisApp_UpdateFrontSteerHold(0.001f);
    CHECK(g_motor_6020_1.cmd == 0);
    printf("Front steering zero hold: %u checks, %u failures\n", checks, failures);
    return failures == 0u ? 0 : 1;
}
