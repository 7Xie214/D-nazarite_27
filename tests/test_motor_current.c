#include "Motor_Test.h"
#include "My_CAN.h"

#include <stdio.h>
#include <string.h>

static FDCAN_HandleTypeDef bus;
static FDCAN_HandleTypeDef *sent_bus;
static uint16_t sent_id;
static uint8_t sent_data[8];
static unsigned int calls;
static unsigned int failures;
static HAL_StatusTypeDef send_result = HAL_OK;

HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *hfdcan,
                               uint16_t id, const uint8_t *data, uint8_t len)
{
    sent_bus = hfdcan;
    sent_id = id;
    if (len != 8u) { failures++; }
    memcpy(sent_data, data, sizeof(sent_data));
    calls++;
    return send_result;
}

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL line %d: %s\n", __LINE__, #condition); \
        failures++; \
    } \
} while (0)

int main(void)
{
    static const int16_t currents[] = {1000, -1000, 0, 16384, -10000};
    uint8_t id, byte;
    size_t c;
    for (id = 1u; id <= 8u; id++) {
        for (c = 0u; c < sizeof(currents) / sizeof(currents[0]); c++) {
            const uint16_t raw = (uint16_t)currents[c];
            const uint8_t slot = (uint8_t)((id - 1u) % 4u);
            const unsigned int before = calls;
            CHECK(Test_MotorCurrent(&bus, id, currents[c]) == HAL_OK);
            CHECK(calls == before + 1u);
            CHECK(sent_bus == &bus);
            CHECK(sent_id == ((id <= 4u) ? 0x200u : 0x1FFu));
            for (byte = 0u; byte < 8u; byte++) {
                uint8_t expected = 0u;
                if (byte == slot * 2u) { expected = (uint8_t)(raw >> 8u); }
                if (byte == slot * 2u + 1u) { expected = (uint8_t)(raw & 0xFFu); }
                CHECK(sent_data[byte] == expected);
            }
        }
    }
    {
        const unsigned int before = calls;
        CHECK(Test_MotorCurrent(&bus, 0u, 1000) == HAL_ERROR);
        CHECK(Test_MotorCurrent(&bus, 9u, 1000) == HAL_ERROR);
        CHECK(calls == before);
    }
    send_result = HAL_BUSY;
    CHECK(Test_MotorCurrent(&bus, 1u, 1000) == HAL_BUSY);
    {
        /* Literal frames from the GM6020 voltage protocol. Catch reuse of
         * C620 frame IDs, wrong group/slot, byte order and sign encoding. */
        static const struct {
            uint8_t id;
            int16_t voltage;
            uint16_t frame;
            uint8_t data[8];
        } cases[] = {
            {1u,   2000, 0x1FFu, {0x07, 0xD0, 0, 0, 0, 0, 0, 0}},
            {2u,  -2000, 0x1FFu, {0, 0, 0xF8, 0x30, 0, 0, 0, 0}},
            {3u,  30000, 0x1FFu, {0, 0, 0, 0, 0x75, 0x30, 0, 0}},
            {4u, -30000, 0x1FFu, {0, 0, 0, 0, 0, 0, 0x8A, 0xD0}},
            {5u,   2000, 0x2FFu, {0x07, 0xD0, 0, 0, 0, 0, 0, 0}},
            {6u,  -2000, 0x2FFu, {0, 0, 0xF8, 0x30, 0, 0, 0, 0}},
            {7u,  30000, 0x2FFu, {0, 0, 0, 0, 0x75, 0x30, 0, 0}},
            {1u,      0, 0x1FFu, {0, 0, 0, 0, 0, 0, 0, 0}},
            {7u, -30000, 0x2FFu, {0, 0, 0, 0, 0x8A, 0xD0, 0, 0}}
        };
        send_result = HAL_OK;
        for (c = 0u; c < sizeof(cases) / sizeof(cases[0]); c++) {
            const unsigned int before = calls;
            CHECK(Test_GM6020Voltage(&bus, cases[c].id, cases[c].voltage) == HAL_OK);
            CHECK(calls == before + 1u && sent_bus == &bus);
            CHECK(sent_id == cases[c].frame);
            CHECK(memcmp(sent_data, cases[c].data, sizeof(sent_data)) == 0);
        }
        {
            const unsigned int before = calls;
            CHECK(Test_GM6020Voltage(&bus, 0u, 2000) == HAL_ERROR);
            CHECK(Test_GM6020Voltage(&bus, 8u, 2000) == HAL_ERROR);
            CHECK(Test_GM6020Voltage(&bus, 255u, 2000) == HAL_ERROR);
            CHECK(calls == before);
        }
        send_result = HAL_BUSY;
        CHECK(Test_GM6020Voltage(&bus, 1u, 2000) == HAL_BUSY);
        send_result = HAL_ERROR;
        CHECK(Test_GM6020Voltage(&bus, 1u, 2000) == HAL_ERROR);
        send_result = HAL_TIMEOUT;
        CHECK(Test_GM6020Voltage(&bus, 1u, 2000) == HAL_TIMEOUT);
    }
    printf("40 current + 9 GM6020 voltage packet cases, %u failures\n", failures);
    return (failures == 0u) ? 0 : 1;
}
