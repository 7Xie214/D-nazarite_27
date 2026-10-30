param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskDir = Join-Path ([IO.Path]::GetTempPath()) ('startup-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $taskDir | Out-Null
try {
    [IO.File]::WriteAllText((Join-Path $taskDir 'startup_stubs.h'), @'
#ifndef STARTUP_STUBS_H
#define STARTUP_STUBS_H
#include <stddef.h>
#include <stdint.h>
typedef struct { uint32_t instance; } FDCAN_HandleTypeDef;
typedef struct { uint32_t instance; } UART_HandleTypeDef;
typedef struct { uint32_t LastErrorCode, BusOff; } FDCAN_ProtocolStatusTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
extern UART_HandleTypeDef huart10;
void SCB_EnableICache(void);
void SCB_EnableDCache(void);
void HAL_Init(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);
void MX_FDCAN3_Init(void);
void MX_USART10_UART_Init(void);
void DjiMotor_Init(void);
void Chassis_Init(void);
void HAL_Delay(uint32_t ms);
void Error_Handler(void);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *, FDCAN_ErrorCountersTypeDef *);
#endif
'@)
    [IO.File]::WriteAllText((Join-Path $taskDir 'main.h'), '#include "startup_stubs.h"')
    [IO.File]::WriteAllText((Join-Path $taskDir 'FreeRTOS.h'), '')
    [IO.File]::WriteAllText((Join-Path $taskDir 'usart.h'), '#include "startup_stubs.h"')
    [IO.File]::WriteAllText((Join-Path $taskDir 'cmsis_os2.h'), @'
#include <stdint.h>
#define configTICK_RATE_HZ 1000u
#define osWaitForever 0xffffffffu
void osKernelInitialize(void);
void osKernelStart(void);
uint32_t osKernelGetTickCount(void);
void osDelay(uint32_t ticks);
'@)
    # Keep the complete real startup/main body; MCU clock/MPU register code is
    # separately compiled by Keil. Only replace hardware at its boundaries.
    $taskMain = [IO.File]::ReadAllText((Join-Path $taskRoot 'Core/Src/main.c'))
    $taskEnd = [regex]::Match($taskMain, 'void SystemClock_Config\(void\)\s*\{').Index
    if ($taskEnd -le 0) { throw 'Cannot locate clock implementation.' }
    $taskMain = $taskMain.Substring(0, $taskEnd)
    $taskMain = $taskMain -replace '(?m)^#include .*\r?\n', ''
    $taskMain = $taskMain -replace 'int main\(void\)', 'int Firmware_Main(void)'
    $taskMain = "#include `"startup_stubs.h`"`n#include `"cmsis_os2.h`"`n#include `"My_CAN.h`"`n#include `"My_USART.h`"`n#include `"Motor_Test.h`"`n" + $taskMain + "`nstatic void MPU_Config(void) {}`n"
    $taskArgs = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-Wstrict-prototypes', ('-I'+$taskDir))
    foreach ($taskInclude in @('BSP/Inc','APP/Inc','Test/Inc')) {
        $taskArgs += '-I' + (Join-Path $taskRoot $taskInclude)
    }
    $taskExe = Join-Path $taskDir 'test_startup.exe'
    foreach ($taskMode in @(1, 0)) {
        $taskCaseMain = $taskMain -replace '(?m)^#define MOTOR_CURRENT_TEST_ONLY [01]', ('#define MOTOR_CURRENT_TEST_ONLY ' + $taskMode)
        [IO.File]::WriteAllText((Join-Path $taskDir 'startup_main.c'), $taskCaseMain)
        & $Compiler @taskArgs ('-DTEST_CURRENT_ONLY=' + $taskMode) (Join-Path $taskRoot 'tests/test_startup.c') (Join-Path $taskDir 'startup_main.c') (Join-Path $taskRoot 'APP/Src/Communicate.c') (Join-Path $taskRoot 'Test/Src/Motor_Test.c') -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw 'Startup regression compilation failed.' }
        & $taskExe
        if ($LASTEXITCODE -ne 0) { throw 'Startup regression failed.' }
    }
} finally {
    Get-ChildItem -LiteralPath $taskDir -File | Remove-Item
    Remove-Item -LiteralPath $taskDir
}
