param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskBuildDir = Join-Path ([IO.Path]::GetTempPath()) ('dji-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $taskBuildDir | Out-Null
try {
    [IO.File]::WriteAllText((Join-Path $taskBuildDir 'main.h'), @'
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t instance; } FDCAN_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
uint32_t HAL_GetTick(void);
#endif
'@)
    [IO.File]::WriteAllText((Join-Path $taskBuildDir 'fdcan.h'), @'
#include "main.h"
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
'@)
    [IO.File]::WriteAllText((Join-Path $taskBuildDir 'FreeRTOS.h'), "#define configTICK_RATE_HZ 1000u`n")
    [IO.File]::WriteAllText((Join-Path $taskBuildDir 'cmsis_os2.h'), "#include <stdint.h>`nuint32_t osKernelGetTickCount(void);`n")
    $taskExe = Join-Path $taskBuildDir 'test_dji_motor.exe'
    & $Compiler -std=c99 -Wall -Wextra -Werror -Wconversion -Wshadow -Wstrict-prototypes -pedantic `
        ('-I' + $taskBuildDir) `
        ('-I' + (Join-Path $taskRoot 'Device/Inc')) `
        ('-I' + (Join-Path $taskRoot 'Test/Inc')) `
        ('-I' + (Join-Path $taskRoot 'BSP/Inc')) `
        ('-I' + (Join-Path $taskRoot 'APP/Inc')) `
        ('-I' + (Join-Path $taskRoot 'algorithm/Inc')) `
        (Join-Path $taskRoot 'tests/test_dji_motor.c') `
        (Join-Path $taskRoot 'Device/Src/DjiMotor.c') `
        (Join-Path $taskRoot 'APP/Src/MotorApp.c') `
        (Join-Path $taskRoot 'algorithm/Src/PID_Config.c') `
        (Join-Path $taskRoot 'algorithm/Src/PID.c') `
        (Join-Path $taskRoot 'algorithm/Src/PositionLoop.c') `
        (Join-Path $taskRoot 'algorithm/Src/Traj.c') -o $taskExe
    if ($LASTEXITCODE -ne 0) { throw 'DJI test compilation failed.' }
    foreach ($taskCase in @('capacity', 'protocol', 'init', 'gm_slots', 'startup_feedback', 'app_binding')) {
        & $taskExe $taskCase
        if ($LASTEXITCODE -ne 0) { throw ('DJI tests failed: ' + $taskCase) }
    }
} finally {
    foreach ($taskFile in @('main.h', 'fdcan.h', 'FreeRTOS.h', 'cmsis_os2.h', 'test_dji_motor.exe')) {
        $taskPath = Join-Path $taskBuildDir $taskFile
        if (Test-Path -LiteralPath $taskPath) { Remove-Item -LiteralPath $taskPath }
    }
    Remove-Item -LiteralPath $taskBuildDir
}
