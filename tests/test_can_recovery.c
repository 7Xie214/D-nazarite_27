#include "My_CAN.h"
#include "fdcan.h"
#undef CRC /* Windows headers also use CRC as a field name. */
#include <windows.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
static unsigned int failures, aborts;
static uint32_t notifications;
static uint32_t tick = 100u;
uint32_t HAL_GetTick(void) { return tick; }
#define CHECK(c) do { if (!(c)) { printf("FAIL %d: %s\n", __LINE__, #c); failures++; } } while (0)

/* Match HAL weak defaults, so the old BSP links but cannot recover. */
__weak void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *can) { (void)can; }
__weak void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *can, uint32_t flags)
{ (void)can; (void)flags; }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *can,
    uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{ (void)can; (void)a; (void)b; (void)c; (void)d; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *can, const FDCAN_FilterTypeDef *filter)
{ (void)can; (void)filter; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *can, uint32_t flags, uint32_t buffers)
{ (void)can; (void)buffers; notifications = flags; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *can)
{ can->State = HAL_FDCAN_STATE_BUSY; can->Instance->CCCR &= ~FDCAN_CCCR_INIT; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *can, FDCAN_ProtocolStatusTypeDef *status)
{
    uint32_t psr = can->Instance->PSR;
    memset(status, 0, sizeof(*status));
    status->LastErrorCode = psr & 7u;
    status->BusOff = (psr & FDCAN_PSR_BO) != 0u;
    can->Instance->PSR = (psr & ~7u) | 7u; /* Emulate the LEC read side effect. */
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *can, FDCAN_ErrorCountersTypeDef *counters)
{
    memset(counters, 0, sizeof(*counters));
    counters->TxErrorCnt = can->Instance->ECR & 0xffu;
    counters->RxErrorCnt = (can->Instance->ECR >> 8u) & 0x7fu;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *can, uint32_t buffers)
{
    CHECK((can->Instance->CCCR & FDCAN_CCCR_INIT) != 0u);
    can->Instance->TXBCR = buffers;
    aborts++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *can,
    const FDCAN_TxHeaderTypeDef *header, const uint8_t *data)
{
    (void)header; (void)data;
    return (can->Instance->PSR & FDCAN_PSR_BO) ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *can, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{ (void)can; (void)fifo; (void)header; (void)data; return HAL_ERROR; }

int main(void)
{
    FDCAN_HandleTypeDef *buses[3] = {&hfdcan1, &hfdcan2, &hfdcan3};
    FDCAN_GlobalTypeDef *instances[3] = {FDCAN1, FDCAN2, FDCAN3};
    const CAN_Filter_t filter = {0x201u, 0x7ffu};
    const uint8_t data[8] = {7u, 0xd0u, 0u, 0u, 0u, 0u, 0u, 0u};
    unsigned int i;
    void *mapped = VirtualAlloc((void *)0x40000000u, 0x10000u,
        MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if (mapped != (void *)0x40000000u) { puts("Cannot map test register range"); return 2; }
    for (i = 0u; i < 3u; i++) {
        FDCAN_HandleTypeDef *can = buses[i];
        can->Instance = instances[i];
        can->State = HAL_FDCAN_STATE_READY;
        can->Init.StdFiltersNbr = 32u;
        CHECK(BSP_CAN_Init(can, &filter, 1u) == HAL_OK);
        CHECK((notifications & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0u);
        CHECK((notifications & FDCAN_IT_BUS_OFF) != 0u);
        CHECK((notifications & FDCAN_IT_ARB_PROTOCOL_ERROR) != 0u);

        CHECK(g_can_diagnostics[i].first_error_code == 0u);
        can->Instance->PSR = 3u; /* First failure: missing ACK. */
        can->Instance->ECR = 8u;
        HAL_FDCAN_ErrorCallback(can);
        CHECK(g_can_diagnostics[i].first_error_code == 3u);
        CHECK(g_can_diagnostics[i].error_record_count == 1u);
        CHECK(g_can_diagnostics[i].last_error_tick_ms == 100u);
        can->Instance->PSR = 7u; /* A later diagnostic read must not erase it. */
        HAL_FDCAN_ErrorCallback(can);
        CHECK(g_can_diagnostics[i].first_error_code == 3u);
        CHECK(g_can_diagnostics[i].last_error_code == 3u);
        CHECK(g_can_diagnostics[i].last_ecr == 8u);
        CHECK(g_can_diagnostics[i].error_record_count == 1u);

        can->Instance->PSR = FDCAN_PSR_BO | 7u;
        can->Instance->ECR = 0xf8u;
        can->Instance->CCCR = FDCAN_CCCR_INIT;
        can->Instance->TXBRP = 0x300u;
        can->Instance->TXBCR = 0u;
        CHECK(BSP_CAN_Send(can, 0x200u, data, 8u) == HAL_ERROR);
        HAL_FDCAN_ErrorStatusCallback(can, FDCAN_IT_BUS_OFF);
        CHECK((can->Instance->CCCR & FDCAN_CCCR_INIT) == 0u);
        CHECK(can->Instance->TXBCR == 0x300u);
        CHECK(aborts == i + 1u);
        CHECK(can->State == HAL_FDCAN_STATE_BUSY);
        CHECK(g_can_diagnostics[i].bus_off_count == 1u);
        CHECK(g_can_diagnostics[i].first_error_code == 3u);
        CHECK(g_can_diagnostics[i].last_error_code == 3u);
        CHECK(g_can_diagnostics[i].last_ecr == 0xf8u);

        /* Exiting Bus-Off is a hardware sequence, not an immediate PSR write. */
        CHECK((can->Instance->PSR & FDCAN_PSR_BO) != 0u);
        can->Instance->PSR = 0u;
        can->Instance->TXBRP = 0u;
        CHECK(BSP_CAN_Send(can, 0x200u, data, 8u) == HAL_OK);
        can->Instance->CCCR = FDCAN_CCCR_INIT;
        HAL_FDCAN_ErrorStatusCallback(can, FDCAN_IT_BUS_OFF); /* Exit notification */
        CHECK((can->Instance->CCCR & FDCAN_CCCR_INIT) != 0u);
        CHECK(aborts == i + 1u);
        CHECK(g_can_diagnostics[i].bus_off_count == 1u);
        CHECK((g_can_diagnostics[i].last_psr & FDCAN_PSR_BO) != 0u);
        can->Instance->PSR = 5u;
        tick = 150u;
        HAL_FDCAN_ErrorCallback(can);
        CHECK(g_can_diagnostics[i].first_error_code == 3u);
        CHECK(g_can_diagnostics[i].last_error_code == 5u);
        CHECK(g_can_diagnostics[i].last_error_tick_ms == 150u);
        CHECK(g_can_diagnostics[i].error_record_count == 3u);
        can->Instance->PSR = 0u;
        HAL_FDCAN_ErrorCallback(can);
        CHECK(g_can_diagnostics[i].last_error_code == 5u);
        HAL_FDCAN_ErrorStatusCallback(can, FDCAN_IT_ERROR_PASSIVE);
        CHECK(aborts == i + 1u);
        tick = 100u;
    }
    VirtualFree(mapped, 0u, MEM_RELEASE);
    printf("CAN recovery checks: %u failures\n", failures);
    return failures != 0u;
}
