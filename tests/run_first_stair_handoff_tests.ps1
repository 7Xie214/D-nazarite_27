param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskDir = Join-Path ([IO.Path]::GetTempPath()) ('handoff-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $taskDir | Out-Null
try {
    [IO.File]::WriteAllText((Join-Path $taskDir 'main.h'), @'
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t instance; } FDCAN_HandleTypeDef;
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
uint32_t HAL_GetTick(void);
#endif
'@)
    [IO.File]::WriteAllText((Join-Path $taskDir 'FreeRTOS.h'), "#define configTICK_RATE_HZ 1000u`n")
    [IO.File]::WriteAllText((Join-Path $taskDir 'cmsis_os2.h'), "#ifndef TEST_CMSIS_H`n#define TEST_CMSIS_H`n#include <stdint.h>`ntypedef enum {osOK=0, osErrorResource=-3} osStatus_t;`nuint32_t osKernelGetTickCount(void);`nvoid osDelay(uint32_t ticks);`n#endif`n")
    [IO.File]::WriteAllText((Join-Path $taskDir 'fdcan.h'), "#include `"main.h`"`nextern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;`n")
    $taskArgs = @('-std=c99','-Wall','-Wextra','-Werror','-Wconversion','-Wshadow','-Wstrict-prototypes','-pedantic',('-I'+$taskDir))
    foreach ($taskInclude in @('Task/Inc','APP/Inc','Device/Inc','algorithm/Inc','BSP/Inc','Test/Inc')) {
        $taskArgs += '-I' + (Join-Path $taskRoot $taskInclude)
    }
    $taskExe = Join-Path $taskDir 'handoff.exe'
    foreach ($taskOptimization in @('-O0','-O2')) {
        & $Compiler @taskArgs $taskOptimization (Join-Path $taskRoot 'tests/test_first_stair_handoff.c') (Join-Path $taskRoot 'Task/Src/My_task.c') -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw 'First stair handoff compilation failed.' }
        foreach ($taskCase in @('lift_late','pair_late','lift_stalled','lift_fault','fallback','retry','wrap')) {
            & $taskExe $taskCase
            if ($LASTEXITCODE -ne 0) { throw ('First stair handoff failed: ' + $taskCase) }
        }
    }
} finally {
    Get-ChildItem -LiteralPath $taskDir -File | Remove-Item
    Remove-Item -LiteralPath $taskDir
}
