#include "MotorApp.h"
#include "My_CAN.h"
#include "Encoder_Test.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick = 100u;
static unsigned int failures;
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); failures++; } } while (0)
uint32_t HAL_GetTick(void) { return tick; }
uint32_t osKernelGetTickCount(void) { return tick; }
void Test_EncoderOnCanFrame(FDCAN_HandleTypeDef *can, uint16_t id,
                            const uint8_t *data, uint8_t len)
{ (void)can; (void)id; (void)data; (void)len; }
HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *can, uint16_t id,
                              const uint8_t *data, uint8_t len)
{ (void)data; CHECK(can == &hfdcan3); CHECK(id == 0x200u || id == 0x1FFu); CHECK(len == 8u); return HAL_OK; }

static void feedback(unsigned int index, uint16_t angle, int16_t speed)
{
    Axis_t *axis = MotorApp_GetAxis((uint8_t)index);
    uint16_t raw = (uint16_t)speed;
    uint8_t data[8] = {(uint8_t)(angle >> 8), (uint8_t)angle,
                       (uint8_t)(raw >> 8), (uint8_t)raw, 0, 0, 30, 0};
    DjiMotor_OnCanFrame(&hfdcan3, (uint16_t)(0x200u + axis->motor->id), data, 8u);
}
static void stationary(void)
{
    unsigned int i;
    tick++;
    for (i = 0u; i < 5u; i++) { feedback(i, 0u, 0); }
    MotorApp_Update(0.001f);
}

int main(int argc, char **argv)
{
    unsigned int i, n;
    float targets[3];
    float turns[3] = {0.125f, -0.25f, 0.0625f};
    (void)argc;
    DjiMotor_Init();
    MotorApp_Init();
    for (i = 0u; i < 5u; i++) { feedback(i, 0u, 0); }
    MotorApp_Update(0.001f);
    if (strcmp(argv[1], "speed_units") == 0) {
        const float peaks[3] = {20.0f, 4.2f, 20.0f}; /* Faster shared moves / ID5 single reference. */
        for (i = 0u; i < 3u; i++) {
            Cascade_t control = MotorApp_GetAxis((uint8_t)i)->cascade;
            float scale = MotorApp_GetAxis((uint8_t)i)->motor->model->gear_ratio * 60.0f / 6.28318530718f;
            float output;
            control.spd.Ki = 0.0f;
            control.spd.Kp = 1467.0171f; /* Fixed unit-conversion fixture; user may tune actual gains. */
            /* A 130 rotor-rpm error must reproduce the original P output:
             * (150 - 20) * 8 = 1040, despite SI-unit feedback. */
            output = Cascade_Update(&control, 0.0f, 20.0f / scale, 0.0f, 150.0f / scale, 0.001f);
            CHECK(fabsf(output - 1040.0f) < 0.02f);
            output = Cascade_Update(&control, 0.0f, peaks[i], 0.0f, peaks[i], 0.001f);
            CHECK(fabsf(control.spd.setpoint - peaks[i]) < 1e-5f);
            CHECK(fabsf(output) < 0.02f); /* No clipping of the faster feedforward speed. */
        }
    } else if (strcmp(argv[1], "lift_fast") == 0) {
        const float lift_turns[4] = {-20.0f, 20.0f, -35.0f, 35.0f};
        Axis_t *axis = MotorApp_GetAxis(0u);
        for (i = 0u; i < 4u; i++) {
            float peak = 0.0f;
            CHECK(MotorApp_MoveTurns(axis, lift_turns[i]) == MOTORAPP_OK);
            for (n = 0u; n < 4000u && axis->traj.active != 0u; n++) {
                stationary();
                if (fabsf(axis->traj.vel_ref) > peak) { peak = fabsf(axis->traj.vel_ref); }
            }
            printf("ID5 %.1f rotor turns: reference %.3f s, peak %.3f rad/s\n",
                   (double)lift_turns[i], (double)n * 0.001, (double)peak);
            CHECK(n <= (i < 2u ? 650u : 900u));
            CHECK(axis->cascade.pos.out_max >= peak * 1.2f); /* room for position correction */
            CHECK(axis->state == AXIS_MOVING); /* reference completion does not fake actual arrival */
        }
    } else if (strcmp(argv[1], "single") == 0) {
        Axis_t *axis = MotorApp_GetAxis(0u);
        CHECK(MotorApp_MoveTurns(axis, 2.0f) == MOTORAPP_OK);
        CHECK(fabsf(axis->traj.target - 2.0f * 6.28318530718f / axis->motor->model->gear_ratio) < 1e-5f);
        CHECK(MotorApp_MoveTurns(MotorApp_GetAxis(3u), -1.0f) == MOTORAPP_OK);
        CHECK(fabsf(MotorApp_GetAxis(3u)->traj.target + 6.28318530718f / 36.0f) < 1e-5f);
        CHECK(MotorApp_MoveTurns(axis, 0.125f) == MOTORAPP_OK);
        axis->traj.pos_ref = axis->traj.target;
        Traj_Stop(&axis->traj);
        for (i = 0u; i < 100u; i++) { stationary(); }
        CHECK(axis->state == AXIS_MOVING); /* Planned stop alone is not arrival. */
        for (i = 0u; i < 100u; i++) {
            tick++; feedback(0u, 1024u, 0); MotorApp_Update(0.001f);
        }
        CHECK(axis->state == AXIS_IDLE);
        CHECK(fabsf(axis->traj.pos_ref - axis->traj.target) < 1e-6f);
    } else {
        if (strcmp(argv[1], "zero") == 0) { turns[2] = 0.0f; }
        if (strcmp(argv[1], "zero_all") == 0) {
            turns[0] = 0.0f; turns[1] = 0.0f; turns[2] = 0.0f;
        }
        if (strcmp(argv[1], "atomic") == 0) {
            MotorApp_GetAxis(2u)->motor->rx_count = 0u;
            CHECK(MotorApp_Move3508TurnsSync(turns, 1.0f, 1.0f) == MOTORAPP_ERR_NOFB);
            CHECK(MotorApp_GetAxis(0u)->state == AXIS_IDLE);
            CHECK(MotorApp_GetAxis(1u)->state == AXIS_IDLE);
            feedback(2u, 0u, 0);
        }
        CHECK(MotorApp_Move3508TurnsSync(turns, 1.0f, 1.0f) == MOTORAPP_OK);
        CHECK(MotorApp_Move3508TurnsSync(turns, 1.0f, 1.0f) == MOTORAPP_ERR_BUSY);
        CHECK(MotorApp_MoveTurns(MotorApp_GetAxis(0u), 1.0f) == MOTORAPP_ERR_BUSY);
        for (i = 0u; i < 3u; i++) { targets[i] = MotorApp_GetAxis((uint8_t)i)->traj.target; }
        if (strcmp(argv[1], "fault") == 0) {
            tick += 51u;
            feedback(0u, 0u, 0); feedback(2u, 0u, 0);
            MotorApp_Update(0.001f);
            for (i = 0u; i < 3u; i++) {
                CHECK(MotorApp_GetAxis((uint8_t)i)->state == AXIS_FAULT);
                CHECK(MotorApp_GetAxis((uint8_t)i)->motor->cmd == 0);
            }
            stationary();
            for (i = 0u; i < 3u; i++) {
                CHECK(MotorApp_GetAxis((uint8_t)i)->state == AXIS_IDLE);
                CHECK(MotorApp_GetAxis((uint8_t)i)->traj.target == 0.0f);
            }
        } else if (strcmp(argv[1], "zero_all") == 0) {
            for (n = 0u; n < 100u; n++) { stationary(); }
            for (i = 0u; i < 3u; i++) {
                CHECK(MotorApp_GetAxis((uint8_t)i)->state == AXIS_IDLE);
                CHECK(MotorApp_GetAxis((uint8_t)i)->traj.target == 0.0f);
            }
        } else {
            for (n = 0u; n < 4000u; n++) {
                stationary();
                for (i = 1u; i < 3u; i++) {
                    CHECK(MotorApp_GetAxis((uint8_t)i)->traj.active == MotorApp_GetAxis(0u)->traj.active);
                    if (targets[i] == 0.0f) {
                        CHECK(MotorApp_GetAxis((uint8_t)i)->traj.pos_ref == 0.0f);
                    } else {
                        CHECK(fabsf(MotorApp_GetAxis((uint8_t)i)->traj.pos_ref / targets[i] -
                                    MotorApp_GetAxis(0u)->traj.pos_ref / targets[0]) < 1e-5f);
                    }
                }
                if (MotorApp_GetAxis(0u)->traj.active == 0u) { break; }
            }
            CHECK(n < 4000u);
            for (i = 0u; i < 3u; i++) {
                if (fabsf(turns[i]) > MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS) {
                    CHECK(MotorApp_GetAxis((uint8_t)i)->state == AXIS_MOVING);
                }
            }
            for (n = 0u; n < 100u; n++) {
                tick++; feedback(0u, 1024u, 100); feedback(1u, 6144u, -100);
                feedback(2u, turns[2] == 0.0f ? 0u : 512u, 100);
                MotorApp_Update(0.001f);
            }
            for (i = 0u; i < 3u; i++) {
                if (fabsf(turns[i]) > MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS) {
                    CHECK(MotorApp_GetAxis((uint8_t)i)->state == AXIS_MOVING);
                }
            }
            for (n = 0u; n < 100u; n++) {
                int16_t speed = n == 30u ? 100 : 0;
                tick++; feedback(0u, 1024u, speed); feedback(1u, 6144u, speed);
                feedback(2u, turns[2] == 0.0f ? 0u : 512u, speed);
                MotorApp_Update(0.001f);
                if (n == 60u && MOTORAPP_SETTLE_TIME_S > 0.03f) {
                    for (i = 0u; i < 3u; i++) { CHECK(MotorApp_GetAxis((uint8_t)i)->state == AXIS_MOVING); }
                }
            }
            for (i = 0u; i < 3u; i++) {
                CHECK(MotorApp_GetAxis((uint8_t)i)->state == AXIS_IDLE);
                CHECK(MotorApp_GetAxis((uint8_t)i)->traj.target == targets[i]);
            }
        }
    }
    printf("Upper motion %s: %u failures\n", argv[1], failures);
    return failures == 0u ? 0 : 1;
}
