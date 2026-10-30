#include "Encoder_Test.h"

#include <stdio.h>

uint32_t test_irq_mask;
static FDCAN_HandleTypeDef bus1, bus2, other_bus;
static unsigned int checks, failures;

#define CHECK(condition) do { \
    checks++; \
    if (!(condition)) { \
        failures++; \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
    } \
} while (0)

static void feedback(FDCAN_HandleTypeDef *bus, uint8_t motor_id, uint16_t angle)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(angle >> 8u);
    data[1] = (uint8_t)(angle & 0xFFu);
    Test_EncoderOnCanFrame(bus, (uint16_t)(0x204u + motor_id), data, 8u);
}

int main(void)
{
    uint8_t short_frame[8] = {0};
    feedback(&bus1, 1u, 100u);
    CHECK(g_encoder_calibration_mask == 0u);

    /* 3/1 on CAN1, 4/2 on CAN2: sort the ID and bus together. */
    CHECK(Test_EncoderCalibrate(&bus1, &bus2, 3u, 1u, 4u, 2u) == HAL_OK);
    CHECK(test_irq_mask == 0u);
    CHECK(g_encoder_calibration_mask == 0u);
    feedback(&bus2, 1u, 999u);
    feedback(&other_bus, 2u, 999u);
    feedback(&bus1, 7u, 999u);
    CHECK(g_encoder_calibration_mask == 0u);
    Test_EncoderOnCanFrame(&bus1, 0x205u, short_frame, 1u);
    Test_EncoderOnCanFrame(&bus1, 0x205u, NULL, 8u);
    CHECK(g_encoder_calibration_mask == 0u);
    feedback(&bus1, 1u, 0u);        /* Zero is a valid encoder result. */
    CHECK(g_encoder_calibration[0] == 0u);
    CHECK(g_encoder_calibration_mask == 0x01u);
    feedback(&bus2, 2u, 1234u);
    feedback(&bus1, 3u, 8191u);
    feedback(&bus2, 4u, 4567u);
    CHECK(g_encoder_calibration[1] == 1234u);
    CHECK(g_encoder_calibration[2] == 8191u);
    CHECK(g_encoder_calibration[3] == 4567u);
    CHECK(g_encoder_calibration_mask == 0x0Fu);
    feedback(&bus1, 3u, 88u);
    CHECK(g_encoder_calibration[2] == 8191u); /* Freeze each first sample. */

    /* Equal IDs on different buses remain separate; CAN1 comes first on ties. */
    test_irq_mask = 1u;
    CHECK(Test_EncoderCalibrate(&bus1, &bus2, 2u, 1u, 2u, 1u) == HAL_OK);
    CHECK(test_irq_mask == 1u);
    test_irq_mask = 0u;
    CHECK(g_encoder_calibration_mask == 0u);
    CHECK(g_encoder_calibration[0] == 0u && g_encoder_calibration[3] == 0u);
    feedback(&bus1, 1u, 11u);
    feedback(&bus2, 1u, 21u);
    feedback(&bus1, 2u, 12u);
    feedback(&bus2, 2u, 22u);
    CHECK(g_encoder_calibration[0] == 11u);
    CHECK(g_encoder_calibration[1] == 21u);
    CHECK(g_encoder_calibration[2] == 12u);
    CHECK(g_encoder_calibration[3] == 22u);
    CHECK(g_encoder_calibration_mask == 0x0Fu);

    CHECK(Test_EncoderCalibrate(NULL, &bus2, 1u, 2u, 3u, 4u) == HAL_ERROR);
    CHECK(Test_EncoderCalibrate(&bus1, &bus1, 1u, 2u, 3u, 4u) == HAL_ERROR);
    CHECK(Test_EncoderCalibrate(&bus1, &bus2, 1u, 1u, 3u, 4u) == HAL_ERROR);
    CHECK(Test_EncoderCalibrate(&bus1, &bus2, 1u, 2u, 3u, 3u) == HAL_ERROR);
    CHECK(Test_EncoderCalibrate(&bus1, &bus2, 0u, 2u, 3u, 4u) == HAL_ERROR);
    CHECK(Test_EncoderCalibrate(&bus1, &bus2, 1u, 2u, 3u, 8u) == HAL_ERROR);
    CHECK(g_encoder_calibration[3] == 22u);
    CHECK(g_encoder_calibration_mask == 0x0Fu);

    CHECK(Test_EncoderCalibrate(&bus1, &bus2, 1u, 2u, 3u, 7u) == HAL_OK);
    feedback(&bus2, 7u, 8192u);
    CHECK(g_encoder_calibration_mask == 0u);
    feedback(&bus2, 7u, 8191u);
    CHECK(g_encoder_calibration[3] == 8191u);
    CHECK(g_encoder_calibration_mask == 0x08u);
    printf("%u encoder calibration checks, %u failures\n", checks, failures);
    return (failures == 0u) ? 0 : 1;
}
