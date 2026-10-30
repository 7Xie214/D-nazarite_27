param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$taskProjectRoot = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path -LiteralPath (Join-Path $taskProjectRoot 'Test/Inc/Motor_Test.h'))) {
    throw 'FAIL: Test_MotorCurrent has not been implemented.'
}
if (-not (Test-Path -LiteralPath (Join-Path $taskProjectRoot 'Test/Inc/Encoder_Test.h'))) {
    throw 'FAIL: Test_EncoderCalibrate has not been implemented.'
}
$taskBuildDir = Join-Path ([System.IO.Path]::GetTempPath()) ('motor-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $taskBuildDir | Out-Null
$taskHeader = Join-Path $taskBuildDir 'main.h'
$taskExe = Join-Path $taskBuildDir 'test_motor_current.exe'

try {
    # Only replace MCU types; exercise the real packet-building function.
    [System.IO.File]::WriteAllText($taskHeader, @'
#ifndef TEST_STM32_MAIN_H
#define TEST_STM32_MAIN_H
#include <stdint.h>
typedef struct { uint32_t instance; } FDCAN_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
extern uint32_t test_irq_mask;
static inline uint32_t __get_PRIMASK(void) { return test_irq_mask; }
static inline void __disable_irq(void) { test_irq_mask = 1u; }
static inline void __set_PRIMASK(uint32_t value) { test_irq_mask = value; }
#endif
'@)
    foreach ($taskCase in @(
        @{ Test = 'test_motor_current.c'; Source = 'Motor_Test.c' },
        @{ Test = 'test_encoder_calibration.c'; Source = 'Encoder_Test.c' }
    )) {
        & $Compiler -std=c99 -Wall -Wextra -Werror -Wconversion -Wshadow -Wstrict-prototypes -pedantic `
            ('-I' + $taskBuildDir) `
            ('-I' + (Join-Path $taskProjectRoot 'Test/Inc')) `
            ('-I' + (Join-Path $taskProjectRoot 'BSP/Inc')) `
            (Join-Path $taskProjectRoot ('tests/' + $taskCase.Test)) `
            (Join-Path $taskProjectRoot ('Test/Src/' + $taskCase.Source)) -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw ('Test compilation failed: ' + $taskCase.Test) }
        & $taskExe
        if ($LASTEXITCODE -ne 0) { throw ('Tests failed: ' + $taskCase.Test) }
    }
} finally {
    if (Test-Path -LiteralPath $taskExe) { Remove-Item -LiteralPath $taskExe }
    if (Test-Path -LiteralPath $taskHeader) { Remove-Item -LiteralPath $taskHeader }
    Remove-Item -LiteralPath $taskBuildDir
}
