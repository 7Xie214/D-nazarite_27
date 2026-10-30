param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskDir = Join-Path ([IO.Path]::GetTempPath()) ('chassis-task-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $taskDir | Out-Null
try {
    [IO.File]::WriteAllText((Join-Path $taskDir 'main.h'), @'
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t instance; } FDCAN_HandleTypeDef;
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
#endif
'@)
    [IO.File]::WriteAllText((Join-Path $taskDir 'cmsis_os2.h'), @'
#ifndef TEST_CMSIS_H
#define TEST_CMSIS_H
#include <stdint.h>
typedef enum { osOK=0, osError=-1 } osStatus_t;
typedef void *osThreadId_t;
#define osErrorResource ((osStatus_t)-3)
#define osFlagsWaitAny 0u
#define osFlagsError 0x80000000u
#define osFlagsErrorResource 0xFFFFFFFDu
uint32_t osKernelGetTickFreq(void);
uint32_t osKernelGetTickCount(void);
osStatus_t osDelayUntil(uint32_t deadline);
void osThreadExit(void);
int32_t osKernelLock(void);
int32_t osKernelRestoreLock(int32_t old);
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout);
uint32_t osThreadFlagsClear(uint32_t flags);
uint32_t osThreadFlagsSet(osThreadId_t thread, uint32_t flags);
#endif
'@)
    Copy-Item -LiteralPath (Join-Path $taskRoot 'Chassis/Inc/Chassis.h') -Destination $taskDir
    $taskConf = [IO.File]::ReadAllText((Join-Path $taskRoot 'Chassis/Inc/Chassis_Conf.h'))
    $taskExe = Join-Path $taskDir 'test_chassis_task.exe'
    foreach ($taskCase in @('swerve', 'reserved')) {
        $taskCaseConf = $taskConf
        if ($taskCase -eq 'reserved') {
            $taskCaseConf = $taskConf -replace '(?m)^#define CHASSIS_ENABLE_SWERVE .*', '#define CHASSIS_ENABLE_SWERVE 0'
        }
        $taskCaseConf = $taskCaseConf -replace '(?m)^#define CHASSIS_FRONT_LEFT_MOVE_ROTOR_TURNS .*', '#define CHASSIS_FRONT_LEFT_MOVE_ROTOR_TURNS 0.75f'
        $taskCaseConf = $taskCaseConf -replace '(?m)^#define CHASSIS_FRONT_RIGHT_MOVE_ROTOR_TURNS .*', '#define CHASSIS_FRONT_RIGHT_MOVE_ROTOR_TURNS -1.25f'
        [IO.File]::WriteAllText((Join-Path $taskDir 'Chassis_Conf.h'), $taskCaseConf)
        $taskArgs = @('-std=c99','-Wall','-Wextra','-Werror','-Wconversion','-Wshadow','-Wstrict-prototypes','-pedantic',('-I'+$taskDir))
        foreach ($taskInclude in @('Task/Inc','APP/Inc','Chassis/Inc','Device/Inc','algorithm/Inc')) {
            $taskArgs += '-I' + (Join-Path $taskRoot $taskInclude)
        }
        & $Compiler @taskArgs (Join-Path $taskRoot 'tests/test_chassis_task.c') (Join-Path $taskRoot 'Task/Src/ChassisTask.c') -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw ('Task compilation failed: ' + $taskCase) }
        & $taskExe
        if ($LASTEXITCODE -ne 0) { throw ('Task checks failed: ' + $taskCase) }
        if ($taskCase -eq 'swerve') {
            & $Compiler @taskArgs (Join-Path $taskRoot 'tests/test_chassis_notifications.c') (Join-Path $taskRoot 'Task/Src/ChassisTask.c') -o $taskExe
            if ($LASTEXITCODE -ne 0) { throw 'Notification test compilation failed.' }
            & $taskExe
            if ($LASTEXITCODE -ne 0) { throw 'Notification checks failed.' }
        }
    }
} finally {
    foreach ($taskName in @('main.h','cmsis_os2.h','Chassis.h','Chassis_Conf.h','test_chassis_task.exe')) {
        $taskFile = Join-Path $taskDir $taskName
        if (Test-Path -LiteralPath $taskFile) { Remove-Item -LiteralPath $taskFile }
    }
    Remove-Item -LiteralPath $taskDir
}
