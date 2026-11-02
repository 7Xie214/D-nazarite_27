param([string]$Compiler='gcc')
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent $PSScriptRoot
$taskDir=Join-Path ([IO.Path]::GetTempPath()) ('third-stair-tests-'+[guid]::NewGuid())
New-Item -ItemType Directory -Path $taskDir | Out-Null
try {
    [IO.File]::WriteAllText((Join-Path $taskDir 'main.h'), @'
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct {uint32_t instance;} FDCAN_HandleTypeDef;
typedef enum {HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2,HAL_TIMEOUT=3} HAL_StatusTypeDef;
uint32_t HAL_GetTick(void);
#endif
'@)
    [IO.File]::WriteAllText((Join-Path $taskDir 'FreeRTOS.h'), "#define configTICK_RATE_HZ 1000u`n")
    [IO.File]::WriteAllText((Join-Path $taskDir 'cmsis_os2.h'), @'
#ifndef TEST_CMSIS_H
#define TEST_CMSIS_H
#include <stdint.h>
typedef enum {osOK=0,osErrorResource=-3} osStatus_t;
uint32_t osKernelGetTickCount(void);
void osDelay(uint32_t ticks);
#endif
'@)
    $taskArgs=@('-std=c99','-Wall','-Wextra','-Werror','-Wconversion','-Wshadow','-Wstrict-prototypes','-pedantic',('-I'+$taskDir))
    foreach($taskInclude in @('Task/Inc','APP/Inc','Device/Inc','algorithm/Inc')) { $taskArgs+='-I'+(Join-Path $taskRoot $taskInclude) }
    $taskExe=Join-Path $taskDir 'third.exe'
    foreach($taskOpt in @('-O0','-O2')) {
        & $Compiler @taskArgs $taskOpt (Join-Path $taskRoot 'tests/test_third_stair_task.c') (Join-Path $taskRoot 'Task/Src/My_task.c') -o $taskExe
        if($LASTEXITCODE -ne 0){throw 'Third stair task compile failed'}
        foreach($taskCase in @('flow','retry','reject','error','lift_retry','lift_rejected','wait_id5','wait_id1','stale_id5','lift_fault','pair_retry','pair_fault','wait_pair','final_retry','final_rejected','final_fault','wait_final','wait_final_id5','final_fault_id5','stale_final_id5','fourth_retry','fourth_reject','fourth_error','wait_fourth','fourth_lift_retry','fourth_lift_reject','fourth_wait_id5','fourth_wait_id1','fourth_stale_id5','fourth_lift_fault','fourth_pair_retry','fourth_pair_reject','fourth_pair_fault','fourth_wait_pair','end_id1_retry','end_id1_reject','end_id1_wait','end_id1_stale','end_id1_fast','end_id1_active','end_id1_fault','end_front_retry','end_front_reject','end_front_wait','end_front_error','link_id2_retry','link_id2_reject','link_id2_wait','link_id2_stale','link_id2_fast','link_id2_active','link_id2_fault','end_id5_wait','end_id5_stale','end_id5_fault','link_front_wait_after_trigger','resume_retry','link_id2_reset')) {
            & $taskExe $taskCase
            if($LASTEXITCODE -ne 0){throw ('Third stair task failed: '+$taskCase)}
        }
        foreach($taskCase in @('return_retry','return_reject','return_wait','return_stale','return_fast','return_active','return_fault','return_reset')) {
            & $taskExe $taskCase
            if($LASTEXITCODE -ne 0){throw ('Fourth final ID1 return failed: '+$taskCase)}
        }
    }
} finally {
    Get-ChildItem -LiteralPath $taskDir -File | Remove-Item
    Remove-Item -LiteralPath $taskDir
}
