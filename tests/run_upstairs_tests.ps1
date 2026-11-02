param([string]$Compiler = 'gcc', [string[]]$TaskCases = @('wait','start','fault','flow','second_retry','second_fault','timing_wrap','step2_diag','step2_diag_wrap','id5_position','arrival_fallback','fallback_retry','late_2006','fault_step2','fault_pair','half_retry','fault_final','wait_final'), [switch]$MotionOnly)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskSecondMatch = [regex]::Match([IO.File]::ReadAllText((Join-Path $taskRoot 'Task/Src/My_task.c')),
    '\bs_second_stair_id2_rotor_turns\s*=\s*([-+]?\d+(?:\.\d+)?)f\s*;')
if (-not $taskSecondMatch.Success) { throw 'Second-stair ID2 travel configuration not found.' }
$taskSecondDefine = '-DTEST_SECOND_ID2_ROTOR_TURNS=' + $taskSecondMatch.Groups[1].Value + 'f'
$taskDir = Join-Path ([IO.Path]::GetTempPath()) ('upstairs-tests-' + [guid]::NewGuid())
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
    foreach ($taskOptimization in @('-O0','-O2')) {
        $taskExe = Join-Path $taskDir 'upper_motion.exe'
        $taskSources = @('tests/test_upper_motion.c','APP/Src/MotorApp.c','Device/Src/DjiMotor.c',
                         'algorithm/Src/PID_Config.c','algorithm/Src/PID.c','algorithm/Src/PositionLoop.c','algorithm/Src/Traj.c')
        $taskSources = @($taskSources | ForEach-Object { Join-Path $taskRoot $_ })
        & $Compiler @taskArgs $taskOptimization @taskSources -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw 'Upper motion compilation failed.' }
        foreach ($taskCase in @('id2_window','speed_units','idle_drift','lift_fast','single','sync','zero','zero_all','atomic','fault')) {
            & $taskExe $taskCase
            if ($LASTEXITCODE -ne 0) { throw ('Upper motion failed: ' + $taskCase) }
        }
        $taskPairSources = @($taskSources)
        $taskPairSources[0] = Join-Path $taskRoot 'tests/test_2006_pair.c'
        & $Compiler @taskArgs $taskOptimization @taskPairSources -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw '2006 pair compilation failed.' }
        foreach ($taskCase in @('atomic','any_id3','any_id4','any_reverse_id3','any_reverse_id4','timing','timing_wrap','position_finish','early_position','terminal_speed','startup_load','concurrent_id5','rolling_start','zero_offset','settle','clock','blocked_side','no_timeout','gap_id3','gap_id4','fault','fault_id4','sync_reporting','correction','forward','reverse')) {
            & $taskExe $taskCase
            if ($LASTEXITCODE -ne 0) { throw ('2006 pair failed: ' + $taskCase) }
        }
        if ($MotionOnly) { continue }
        $taskExe = Join-Path $taskDir 'upstairs_task.exe'
        & $Compiler @taskArgs $taskOptimization (Join-Path $taskRoot 'tests/test_upstairs_task.c') (Join-Path $taskRoot 'Task/Src/My_task.c') (Join-Path $taskRoot 'algorithm/Src/Traj.c') -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw 'Upstairs task compilation failed.' }
        foreach ($taskCase in $TaskCases) {
            & $taskExe $taskCase
            if ($LASTEXITCODE -ne 0) { throw ('Upstairs task failed: ' + $taskCase) }
        }
        & $Compiler @taskArgs $taskSecondDefine $taskOptimization (Join-Path $taskRoot 'tests/test_second_stair_task.c') (Join-Path $taskRoot 'Task/Src/My_task.c') (Join-Path $taskRoot 'algorithm/Src/Traj.c') -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw 'Full second-stair task compilation failed.' }
        foreach ($taskCase in @('id2_stale','id2_fast_idle','flow','wrap','wait_planner','wait_position','wait_speed','pair_pending','pair_retry','fallback','fallback_retry','pair_fault','extra_fault','id5_fault','wait_extra','id2_retry','wait_id2','id2_fault','id2_rejected','id1_retry','wait_id1','id1_fault','id1_rejected')) {
            & $taskExe $taskCase
            if ($LASTEXITCODE -ne 0) { throw ('Second stair task failed: ' + $taskCase) }
        }
    }
} finally {
    Get-ChildItem -LiteralPath $taskDir -File | Remove-Item
    Remove-Item -LiteralPath $taskDir
}
