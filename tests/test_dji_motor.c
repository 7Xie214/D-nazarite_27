#include "DjiMotor.h"
#include "My_CAN.h"
#include "fdcan.h"
#include "MotorApp.h"

#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
static DjiMotor_t motors[14];
static uint32_t tick = 100u;
static unsigned int checks, failures, send_calls;
static unsigned int kernel_tick_calls;
static DjiMotor_t *refresh_on_tick;
static FDCAN_HandleTypeDef *sent_bus;
static uint16_t sent_id;
static uint8_t sent_data[8];
static HAL_StatusTypeDef send_result = HAL_OK;
#define gm_model DJI_MODEL_6020

uint32_t osKernelGetTickCount(void) { kernel_tick_calls++; return tick; }
uint32_t HAL_GetTick(void)
{
    uint32_t sampled = tick;
    if (refresh_on_tick != NULL) {
        /* CAN ISR arrives after HAL sampled now but before the caller resumes. */
        refresh_on_tick->rx_tick = ++tick;
        refresh_on_tick->rx_count++;
        refresh_on_tick = NULL;
    }
    return sampled;
}

void Test_EncoderOnCanFrame(FDCAN_HandleTypeDef *bus, uint16_t id,
                           const uint8_t *data, uint8_t len)
{
    (void)bus; (void)id; (void)data; (void)len;
}

HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *bus, uint16_t id,
                               const uint8_t *data, uint8_t len)
{
    sent_bus = bus;
    sent_id = id;
    if (len != 8u) { failures++; }
    memcpy(sent_data, data, sizeof(sent_data));
    send_calls++;
    return send_result;
}

#define CHECK(condition) do { \
    checks++; \
    if (!(condition)) { \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
        failures++; \
    } \
} while (0)

static void test_capacity(void)
{
    uint8_t i;
    CHECK(DJI_MOTOR_MAX == 13u);
    for (i = 0u; i < 13u; i++) {
        FDCAN_HandleTypeDef *bus = (i < 5u) ? &hfdcan1 :
                                   ((i < 9u) ? &hfdcan2 : &hfdcan3);
        uint8_t id = (uint8_t)((i < 5u) ? i + 1u :
                               ((i < 9u) ? i - 4u : i - 8u));
        const DjiMotor_Model_t *model = (i >= 9u) ? &gm_model :
                                         ((i == 3u || i == 4u) ? &DJI_MODEL_2006 : &DJI_MODEL_3508);
        CHECK(DjiMotor_Register(&motors[i], bus, id, model) == &motors[i]);
    }
    CHECK(DjiMotor_Register(&motors[13], &hfdcan3, 5u, &DJI_MODEL_3508) == NULL);
    CHECK(g_djimotor_reg_fail == 1u);
}

static void feedback(FDCAN_HandleTypeDef *bus, uint16_t id)
{
    static const uint8_t data[8] = {0x10u, 0x00u, 0xFFu, 0x88u,
                                     0xFEu, 0x38u, 42u, 0u};
    DjiMotor_OnCanFrame(bus, id, data, 8u);
}

static void expect_packet(FDCAN_HandleTypeDef *bus, uint8_t group,
                          uint16_t id, int16_t a, int16_t b, int16_t c, int16_t d)
{
    const int16_t values[4] = {a, b, c, d};
    uint8_t i;
    unsigned int before = send_calls;
    CHECK(DjiMotor_SendGroup(bus, group) == send_result);
    CHECK(send_calls == before + 1u);
    CHECK(sent_bus == bus);
    CHECK(sent_id == id);
    for (i = 0u; i < 4u; i++) {
        uint16_t raw = (uint16_t)values[i];
        CHECK(sent_data[i * 2u] == (uint8_t)(raw >> 8u));
        CHECK(sent_data[i * 2u + 1u] == (uint8_t)(raw & 0xFFu));
    }
}

static void test_protocol(void)
{
    static const uint8_t data[8] = {0u};
    unsigned int before;
    uint8_t i;
    CHECK(DjiMotor_Register(&motors[0], &hfdcan1, 1u, &DJI_MODEL_3508) == &motors[0]);
    CHECK(DjiMotor_Register(&motors[1], &hfdcan1, 1u, &gm_model) == &motors[1]);
    CHECK(DjiMotor_Register(&motors[2], &hfdcan1, 6u, &DJI_MODEL_3508) == &motors[2]);
    CHECK(DjiMotor_Register(&motors[3], &hfdcan1, 3u, &gm_model) == &motors[3]);
    CHECK(DjiMotor_Register(&motors[4], &hfdcan2, 7u, &gm_model) == &motors[4]);
    CHECK(DjiMotor_Register(&motors[5], &hfdcan2, 8u, &DJI_MODEL_2006) == &motors[5]);
    CHECK(DjiMotor_Register(&motors[6], &hfdcan3, 8u, &DJI_MODEL_3508) == &motors[6]);
    CHECK(DjiMotor_Register(&motors[7], &hfdcan1, 2u, &gm_model) == NULL);
    CHECK(DjiMotor_Register(&motors[7], &hfdcan1, 5u, &DJI_MODEL_3508) == NULL);
    CHECK(DjiMotor_Register(&motors[0], &hfdcan3, 3u, &DJI_MODEL_3508) == NULL);
    CHECK(motors[0].hfdcan == &hfdcan1);
    CHECK(motors[0].id == 1u);
    CHECK(DjiMotor_Register(&motors[7], &hfdcan3, 0u, &DJI_MODEL_3508) == NULL);
    CHECK(DjiMotor_Register(&motors[7], &hfdcan3, 9u, &DJI_MODEL_3508) == NULL);
    CHECK(DjiMotor_Register(&motors[7], &hfdcan3, 8u, &gm_model) == NULL);
    CHECK(DjiMotor_Register(NULL, &hfdcan3, 1u, &gm_model) == NULL);
    CHECK(DjiMotor_Register(&motors[7], NULL, 1u, &gm_model) == NULL);
    CHECK(DjiMotor_Register(&motors[7], &hfdcan3, 1u, NULL) == NULL);
    feedback(&hfdcan1, 0x205u);
    CHECK(motors[1].rx_count == 1u);
    CHECK(motors[0].rx_count == 0u);
    feedback(&hfdcan3, 0x205u);
    CHECK(motors[1].rx_count == 1u);
    feedback(&hfdcan1, 0x201u);
    feedback(&hfdcan1, 0x206u);
    feedback(&hfdcan1, 0x207u);
    feedback(&hfdcan2, 0x20Bu);
    feedback(&hfdcan2, 0x208u);
    feedback(&hfdcan3, 0x208u);
    for (i = 0u; i < 7u; i++) {
        CHECK(motors[i].rx_count == 1u);
        CHECK(motors[i].raw_angle == 4096u);
        CHECK(motors[i].raw_speed == -120);
        CHECK(motors[i].raw_current == -456);
        CHECK(motors[i].raw_temp == 42u);
    }
    DjiMotor_OnCanFrame(&hfdcan2, 0x20Bu, data, 6u);
    DjiMotor_OnCanFrame(&hfdcan2, 0x20Bu, NULL, 8u);
    CHECK(motors[4].rx_count == 1u);
    DjiMotor_Update(&motors[4]);
    CHECK(motors[4].angle_rad == 0.0f);
    CHECK(motors[4].speed_rad_s < -12.56f && motors[4].speed_rad_s > -12.57f);
    DjiMotor_OnCanFrame(&hfdcan2, 0x20Bu, data, 8u);
    DjiMotor_Update(&motors[4]);
    CHECK(motors[4].angle_rad < -3.14f && motors[4].angle_rad > -3.15f);
    CHECK(motors[4].torque_nm == 0.0f);
    motors[0].cmd = 20000;
    motors[1].cmd = -32768;
    motors[2].cmd = -20000;
    motors[3].cmd = 1234;
    motors[4].cmd = 32767;
    motors[5].cmd = -20000;
    motors[6].cmd = 42;
    expect_packet(&hfdcan1, 0u, 0x200u, 16384, 0, 0, 0);
    expect_packet(&hfdcan1, 1u, 0x1FFu, -30000, -16384, 1234, 0);
    expect_packet(&hfdcan2, 1u, 0x1FFu, 0, 0, 0, -10000);
    expect_packet(&hfdcan2, 2u, 0x2FFu, 0, 0, 30000, 0);
    expect_packet(&hfdcan3, 1u, 0x1FFu, 0, 0, 0, 42);
    tick += 51u;
    expect_packet(&hfdcan1, 1u, 0x1FFu, 0, 0, 0, 0);
    expect_packet(&hfdcan2, 2u, 0x2FFu, 0, 0, 0, 0);
    send_result = HAL_BUSY;
    expect_packet(&hfdcan1, 0u, 0x200u, 0, 0, 0, 0);
    before = send_calls;
    CHECK(DjiMotor_SendGroup(&hfdcan1, 2u) == HAL_ERROR);
    CHECK(DjiMotor_SendGroup(&hfdcan1, 3u) == HAL_ERROR);
    CHECK(DjiMotor_SendGroup(NULL, 0u) == HAL_ERROR);
    CHECK(send_calls == before);
}

static void test_init(void)
{
    /* FL drive/steer, FR drive/steer, RL drive/steer, RR drive/steer. */
    DjiMotor_t *const targets[8] = {
        &g_motor_3508_4, &g_motor_6020_1,
        &g_motor_3508_5, &g_motor_6020_4,
        &g_motor_3508_6, &g_motor_6020_2,
        &g_motor_3508_7, &g_motor_6020_3
    };
    DjiMotor_t *const upper[5] = {
        &g_motor_3508_1, &g_motor_3508_2, &g_motor_3508_3,
        &g_motor_2006_1, &g_motor_2006_2
    };
    static const uint8_t upper_ids[5] = {5u, 2u, 1u, 4u, 3u};
    FDCAN_HandleTypeDef *const buses[8] = {
        &hfdcan1, &hfdcan1, &hfdcan2, &hfdcan2,
        &hfdcan1, &hfdcan1, &hfdcan2, &hfdcan2
    };
    static const uint8_t ids[8] = {1u, 1u, 4u, 4u, 2u, 2u, 3u, 3u};
    static const uint16_t feedback_ids[8] = {
        0x201u, 0x205u, 0x204u, 0x208u, 0x202u, 0x206u, 0x203u, 0x207u
    };
    uint8_t i, j;
    DjiMotor_Init();
    CHECK(g_djimotor_reg_fail == 0u);
    for (i = 0u; i < 5u; i++) {
        CHECK(upper[i]->hfdcan == &hfdcan3);
        CHECK(upper[i]->id == upper_ids[i]);
        CHECK(upper[i]->model == ((i < 3u) ? &DJI_MODEL_3508 : &DJI_MODEL_2006));
    }
    for (i = 0u; i < 8u; i++) {
        CHECK(targets[i]->hfdcan == buses[i]);
        CHECK(targets[i]->id == ids[i]);
        CHECK(targets[i]->model == ((i % 2u == 0u) ? &DJI_MODEL_3508 : &gm_model));
        CHECK(targets[i]->cmd == 0);
        feedback(buses[i], feedback_ids[i]);
        for (j = 0u; j < 8u; j++) {
            CHECK(targets[j]->rx_count == ((j <= i) ? 1u : 0u));
        }
        for (j = 0u; j < 5u; j++) { CHECK(upper[j]->rx_count == 0u); }
        targets[i]->cmd = (int16_t)((i + 1u) * 100u);
    }
    expect_packet(&hfdcan1, 0u, 0x200u, 100, 500, 0, 0);
    expect_packet(&hfdcan1, 1u, 0x1FFu, 200, 600, 0, 0);
    expect_packet(&hfdcan2, 0u, 0x200u, 0, 0, 700, 300);
    expect_packet(&hfdcan2, 1u, 0x1FFu, 0, 0, 800, 400);
    for (i = 0u; i < 5u; i++) {
        feedback(&hfdcan3, (uint16_t)(0x200u + upper_ids[i]));
        for (j = 0u; j < 5u; j++) {
            CHECK(upper[j]->rx_count == ((j <= i) ? 1u : 0u));
        }
        for (j = 0u; j < 8u; j++) { CHECK(targets[j]->rx_count == 1u); }
        upper[i]->cmd = (int16_t)(900u + i * 100u);
    }
    expect_packet(&hfdcan3, 0u, 0x200u, 1100, 1000, 1300, 1200);
    expect_packet(&hfdcan3, 1u, 0x1FFu, 900, 0, 0, 0);
    CHECK(g_djimotor_reg_fail == 0u);
    CHECK(DjiMotor_Register(&motors[13], &hfdcan3, 6u, &DJI_MODEL_3508) == NULL);
}

static void test_gm_slots(void)
{
    uint8_t i, j;
    for (i = 0u; i < 7u; i++) {
        CHECK(DjiMotor_Register(&motors[i], &hfdcan1, (uint8_t)(i + 1u), &gm_model) == &motors[i]);
        feedback(&hfdcan1, (uint16_t)(0x205u + i));
        CHECK(motors[i].rx_count == 1u);
        motors[i].cmd = (i % 2u == 0u) ? 32767 : -32768;
    }
    expect_packet(&hfdcan1, 1u, 0x1FFu, 30000, -30000, 30000, -30000);
    expect_packet(&hfdcan1, 2u, 0x2FFu, 30000, -30000, 30000, 0);
    for (i = 0u; i < 7u; i++) {
        int16_t values[4] = {0, 0, 0, 0};
        for (j = 0u; j < 7u; j++) { motors[j].cmd = 0; }
        motors[i].cmd = -1234;
        values[i % 4u] = -1234;
        expect_packet(&hfdcan1, (i < 4u) ? 1u : 2u,
                      (i < 4u) ? 0x1FFu : 0x2FFu,
                      values[0], values[1], values[2], values[3]);
    }
}

static void test_app_binding(void)
{
    DjiMotor_t *const chassis[8] = {
        &g_motor_3508_4, &g_motor_6020_1, &g_motor_3508_5, &g_motor_6020_4,
        &g_motor_3508_6, &g_motor_6020_2, &g_motor_3508_7, &g_motor_6020_3
    };
    DjiMotor_t *const upper[5] = {
        &g_motor_3508_1, &g_motor_3508_2, &g_motor_3508_3,
        &g_motor_2006_1, &g_motor_2006_2
    };
    uint8_t i;
    /* Main registers once; APP initializers must not register again. */
    DjiMotor_Init();
    MotorApp_Init();
    CHECK(g_djimotor_reg_fail == 0u);
    for (i = 0u; i < 8u; i++) {
        feedback(chassis[i]->hfdcan,
                 (uint16_t)(chassis[i]->model->feedback_base + chassis[i]->id));
        chassis[i]->cmd = (int16_t)(100u + i);
    }
    MotorApp_Update(0.001f);
    CHECK(send_calls == 2u);
    CHECK(sent_bus == &hfdcan3);
    CHECK(sent_id == 0x1FFu);
    for (i = 0u; i < 8u; i++) { CHECK(sent_data[i] == 0u); }
    for (i = 0u; i < AXIS_COUNT; i++) {
        Axis_t *axis = MotorApp_GetAxis(i);
        CHECK(axis != NULL);
        CHECK(axis->motor == upper[i]);
        CHECK(axis->motor->hfdcan == &hfdcan3);
        CHECK(axis->motor->rx_count == 0u);
        CHECK(axis->state == AXIS_FAULT);
        CHECK(MotorApp_MoveTurns(axis, 1.0f) == MOTORAPP_ERR_NOFB);
    }
    for (i = 0u; i < AXIS_COUNT; i++) {
        feedback(&hfdcan3, (uint16_t)(0x200u + upper[i]->id));
    }
    MotorApp_Update(0.001f);
    CHECK(send_calls == 4u);
    CHECK(sent_bus == &hfdcan3);
    for (i = 0u; i < AXIS_COUNT; i++) {
        CHECK(MotorApp_GetAxis(i)->state == AXIS_IDLE);
        CHECK(upper[i]->rx_count == 1u);
    }
    for (i = 0u; i < 8u; i++) {
        CHECK(chassis[i]->rx_count == 1u);
        CHECK(chassis[i]->cmd == (int16_t)(100u + i));
    }
}

static void test_startup_feedback(void)
{
    DjiMotor_Init();
    tick = 500u;
    feedback(&hfdcan3, 0x205u);
    CHECK(g_motor_3508_1.rx_tick == 500u);
    CHECK(g_motor_3508_1.rx_count == 1u);
    CHECK(DjiMotor_IsOnline(&g_motor_3508_1) != 0);
    tick = 550u;
    CHECK(DjiMotor_IsOnline(&g_motor_3508_1) != 0);
    tick = 551u;
    CHECK(DjiMotor_IsOnline(&g_motor_3508_1) == 0);
    tick = UINT32_MAX - 20u;
    feedback(&hfdcan3, 0x205u);
    tick = 20u;
    CHECK(DjiMotor_IsOnline(&g_motor_3508_1) != 0);
    tick = 31u;
    CHECK(DjiMotor_IsOnline(&g_motor_3508_1) == 0);
    tick = 600u;
    feedback(&hfdcan3, 0x203u);
    refresh_on_tick = &g_motor_2006_2;
    CHECK(DjiMotor_IsOnline(&g_motor_2006_2) != 0);
    CHECK(g_motor_2006_2.rx_tick == 601u);
    tick = UINT32_MAX;
    feedback(&hfdcan3, 0x203u);
    refresh_on_tick = &g_motor_2006_2;
    CHECK(DjiMotor_IsOnline(&g_motor_2006_2) != 0);
    CHECK(g_motor_2006_2.rx_tick == 0u);
    CHECK(kernel_tick_calls == 0u); /* Main enables CAN before RTOS startup. */
}

int main(int argc, char **argv)
{
    if ((argc > 1) && (strcmp(argv[1], "protocol") == 0)) {
        test_protocol();
    } else if ((argc > 1) && (strcmp(argv[1], "init") == 0)) {
        test_init();
    } else if ((argc > 1) && (strcmp(argv[1], "gm_slots") == 0)) {
        test_gm_slots();
    } else if ((argc > 1) && (strcmp(argv[1], "app_binding") == 0)) {
        test_app_binding();
    } else if ((argc > 1) && (strcmp(argv[1], "startup_feedback") == 0)) {
        test_startup_feedback();
    } else {
        test_capacity();
    }
    printf("%u DJI checks, %u failures\n", checks, failures);
    return (failures == 0u) ? 0 : 1;
}
