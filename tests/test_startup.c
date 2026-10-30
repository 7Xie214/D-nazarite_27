/* Runs the real main() startup and communication task with MCU/RTOS boundaries
 * replaced. Regressions caught: RTOS object creation breaks bare-metal delay;
 * moving UART setup leaves the communication task without a receive instance. */
#include "startup_stubs.h"
#include "Communicate.h"
#include "My_CAN.h"
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static jmp_buf stop;
static unsigned int failures, sends, delays, uart_inits, receives;
static unsigned int scheduler_running, interrupt_mask, init_failed;
static unsigned int uart_ready;
static unsigned int unrelated_inits;
static unsigned int can_started;
static uint32_t tick;
FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
UART_HandleTypeDef huart10;
extern volatile uint32_t g_test_motor_send_count;
int Firmware_Main(void);

#define CHECK(condition) do { if (!(condition)) { \
    printf("FAIL line %d: %s\n", __LINE__, #condition); failures++; } } while (0)

void SCB_EnableICache(void) {}
void SCB_EnableDCache(void) {}
void HAL_Init(void) {}
void Error_Handler(void) { failures++; longjmp(stop, 1); }
void SystemClock_Config(void) {}
void PeriphCommonClock_Config(void) {}
void MX_GPIO_Init(void) {}
void MX_DMA_Init(void) { unrelated_inits++; }
void MX_FDCAN1_Init(void) {}
void MX_FDCAN2_Init(void) { unrelated_inits++; }
void MX_FDCAN3_Init(void) { unrelated_inits++; }
void MX_USART10_UART_Init(void) { unrelated_inits++; }
void DjiMotor_Init(void) { unrelated_inits++; }
void Chassis_Init(void) { unrelated_inits++; }
void MX_FREERTOS_Init(void) {}
void osKernelInitialize(void) {}
void osKernelStart(void) { scheduler_running = 1u; longjmp(stop, 1); }
uint32_t osKernelGetTickCount(void) { return tick; }

HAL_StatusTypeDef BSP_CAN_Init(FDCAN_HandleTypeDef *can,
                              const CAN_Filter_t *filters, uint8_t count)
{
    (void)filters; (void)count;
    if (can == &hfdcan1) { can_started |= 1u; }
    if (can == &hfdcan2) { can_started |= 2u; }
    if (can == &hfdcan3) { can_started |= 4u; }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *can)
{ CHECK(can == &hfdcan1); return HAL_OK; }
HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *can, uint16_t id,
                              const uint8_t *data, uint8_t len)
{
    /* Current raw-test branch sends CAN3 ID3/-2000, then ID4/+2000.
     * Packet helpers intentionally clear the other slots on each call. */
    CHECK(can == &hfdcan3 && id == 0x200u && len == 8u);
    CHECK(data[0] == 0u && data[1] == 0u && data[2] == 0u && data[3] == 0u);
    if ((sends & 1u) == 0u) {
        CHECK(data[4] == 0xf8u && data[5] == 0x30u && data[6] == 0u && data[7] == 0u);
    } else {
        CHECK(data[4] == 0u && data[5] == 0u && data[6] == 7u && data[7] == 0xd0u);
    }
    sends++;
    return HAL_OK;
}
HAL_StatusTypeDef BSP_USART_Init(UART_HandleTypeDef *uart)
{
    CHECK(uart == &huart10);
    uart_inits++;
    /* Local RVDS port starts uxCriticalNesting at 0xaaaaaaaa. Semaphore
     * creation uses taskENTER/EXIT_CRITICAL; BASEPRI stays set until start. */
    if (scheduler_running == 0u) { interrupt_mask = 1u; }
    uart_ready = (init_failed == 0u);
    return (init_failed != 0u) ? HAL_ERROR : HAL_OK;
}
void HAL_Delay(uint32_t ms)
{
    CHECK(interrupt_mask == 0u);
    if (interrupt_mask != 0u) {
        puts("HAL tick cannot advance: startup created RTOS objects without starting scheduler");
        longjmp(stop, 1);
    }
    tick += ms;
    if (++delays == 3u) { longjmp(stop, 1); }
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *can,
                                            FDCAN_ProtocolStatusTypeDef *status)
{ CHECK(can == &hfdcan1); (void)status; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *can,
                                           FDCAN_ErrorCountersTypeDef *counters)
{ CHECK(can == &hfdcan1); (void)counters; return HAL_OK; }
uint16_t BSP_USART_Recv(UART_HandleTypeDef *uart, uint8_t *buf,
                       uint16_t len, uint32_t timeout)
{
    (void)buf; (void)len; (void)timeout;
    CHECK(uart == &huart10 && uart_ready != 0u);
    receives++;
    longjmp(stop, 1);
    return 0u;
}
HAL_StatusTypeDef BSP_USART_Send(UART_HandleTypeDef *uart, const uint8_t *data,
                                uint16_t len, uint32_t timeout)
{ (void)uart; (void)data; (void)len; (void)timeout; return HAL_OK; }
void osDelay(uint32_t ticks) { (void)ticks; longjmp(stop, 1); }

int main(void)
{
    Bluer_Ctx_t ctx;
    if (setjmp(stop) == 0) { (void)Firmware_Main(); }
    CHECK(uart_inits == 0u);
    CHECK(interrupt_mask == 0u);
    CHECK(can_started == 7u && unrelated_inits != 0u);
#if TEST_CURRENT_ONLY
    CHECK(sends == 6u && g_test_motor_send_count == 3u);
    CHECK(tick == 30u);
    CHECK(scheduler_running == 0u); /* Raw-current test must run before tasks. */
#else
    CHECK(scheduler_running != 0u && sends == 0u);
    CHECK(g_test_motor_send_count == 0u && unrelated_inits != 0u);
#endif

    scheduler_running = 1u;
    memset(&ctx, 0, sizeof(ctx));
    ctx.huart = &huart10;
    if (setjmp(stop) == 0) { Communicate_Task(&ctx); }
    CHECK(uart_inits == 1u && receives == 1u && uart_ready != 0u);

    init_failed = 1u;
    if (setjmp(stop) == 0) { Communicate_Task(&ctx); }
    CHECK(uart_inits == 2u && receives == 1u);
    printf("Startup / UART task regression (current test=%d): %u failures\n",
           TEST_CURRENT_ONLY, failures);
    return (failures == 0u) ? 0 : 1;
}
