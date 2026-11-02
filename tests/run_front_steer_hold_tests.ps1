param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskDir = Join-Path ([IO.Path]::GetTempPath()) ('front-steer-hold-' + [guid]::NewGuid())
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
extern uint32_t test_irq_mask;
static inline uint32_t __get_PRIMASK(void) { return test_irq_mask; }
static inline void __disable_irq(void) { test_irq_mask = 1u; }
static inline void __set_PRIMASK(uint32_t m) { test_irq_mask = m; }
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
#define osFlagsErrorResource 0xfffffffdu
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
    [IO.File]::WriteAllText((Join-Path $taskDir 'fdcan.h'), "#include `"main.h`"`nextern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;`n")
    [IO.File]::WriteAllText((Join-Path $taskDir 'FreeRTOS.h'), "#define configTICK_RATE_HZ 1000u`n")
    Copy-Item -LiteralPath (Join-Path $taskRoot 'Chassis/Inc/Chassis.h') -Destination $taskDir
    $taskConf = [IO.File]::ReadAllText((Join-Path $taskRoot 'Chassis/Inc/Chassis_Conf.h'))
    $taskConf = $taskConf -replace '(?m)^#define CHASSIS_TASK_FRONT_MOVE_ENABLE .*', '#define CHASSIS_TASK_FRONT_MOVE_ENABLE 0'
    [IO.File]::WriteAllText((Join-Path $taskDir 'Chassis_Conf.h'), $taskConf)
    $taskArgs = @('-std=c99','-Wall','-Wextra','-Werror','-Wconversion','-Wshadow','-Wstrict-prototypes','-pedantic',('-I'+$taskDir))
    foreach ($taskInclude in @('Task/Inc','APP/Inc','Chassis/Inc','Device/Inc','algorithm/Inc','BSP/Inc','Test/Inc')) {
        $taskArgs += '-I' + (Join-Path $taskRoot $taskInclude)
    }
    $taskSources = @('tests/test_front_steer_hold.c','Task/Src/ChassisTask.c','APP/Src/ChassisApp.c',
        'Device/Src/DjiMotor.c','Chassis/Src/Chassis.c','Chassis/Src/Swerve.c',
        'algorithm/Src/PID_Config.c','algorithm/Src/PID.c','algorithm/Src/PositionLoop.c','algorithm/Src/Traj.c')
    $taskSources = @($taskSources | ForEach-Object { Join-Path $taskRoot $_ })
    $taskExe = Join-Path $taskDir 'hold.exe'
    foreach ($taskOpt in @('-O0','-O2')) {
        & $Compiler @taskArgs $taskOpt @taskSources -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw 'Front steering hold compilation failed.' }
        & $taskExe
        if ($LASTEXITCODE -ne 0) { throw 'Front steering hold tests failed.' }
    }
} finally {
    Get-ChildItem -LiteralPath $taskDir -File | Remove-Item
    Remove-Item -LiteralPath $taskDir
}
