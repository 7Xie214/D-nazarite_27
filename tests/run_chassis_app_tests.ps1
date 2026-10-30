param([string]$Compiler = 'gcc', [switch]$Optimize)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskBuildDir = Join-Path ([IO.Path]::GetTempPath()) ('chassis-app-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $taskBuildDir | Out-Null
try {
    [IO.File]::WriteAllText((Join-Path $taskBuildDir 'main.h'), @'
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
static inline void __set_PRIMASK(uint32_t mask) { test_irq_mask = mask; }
#endif
'@)
    [IO.File]::WriteAllText((Join-Path $taskBuildDir 'fdcan.h'), "#include `"main.h`"`nextern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;`n")
    [IO.File]::WriteAllText((Join-Path $taskBuildDir 'FreeRTOS.h'), "#define configTICK_RATE_HZ 1000u`n")
    [IO.File]::WriteAllText((Join-Path $taskBuildDir 'cmsis_os2.h'), "#include <stdint.h>`nuint32_t osKernelGetTickCount(void);`n")
    Copy-Item -LiteralPath (Join-Path $taskRoot 'Chassis/Inc/Chassis.h') -Destination $taskBuildDir
    $taskOriginalConf = [IO.File]::ReadAllText((Join-Path $taskRoot 'Chassis/Inc/Chassis_Conf.h'))
    $taskExe = Join-Path $taskBuildDir 'test_chassis_app.exe'
    foreach ($taskCase in @('default', 'configured', 'disabled', 'mecanum', 'omni')) {
        $taskConf = $taskOriginalConf
        if ($taskCase -eq 'configured') {
            foreach ($taskPair in @(
                @('CHASSIS_SWERVE_WHEEL_RADIUS_M', '0.1f'),
                @('CHASSIS_SWERVE_HALF_WHEEL_BASE_M', '0.2f'),
                @('CHASSIS_SWERVE_HALF_TRACK_WIDTH_M', '0.3f'),
                @('CHASSIS_SWERVE_MAX_WHEEL_SPEED_MPS', '2.0f')
            )) {
                $taskConf = $taskConf -replace ('(?m)^#define ' + $taskPair[0] + ' .*'), ('#define ' + $taskPair[0] + ' ' + $taskPair[1])
            }
        } elseif ($taskCase -in @('disabled', 'mecanum', 'omni')) {
            $taskConf = $taskConf -replace '(?m)^#define CHASSIS_ENABLE_SWERVE .*', '#define CHASSIS_ENABLE_SWERVE 0'
            if ($taskCase -eq 'mecanum') {
                $taskConf = $taskConf -replace '(?m)^#define CHASSIS_ENABLE_MECANUM .*', '#define CHASSIS_ENABLE_MECANUM 1'
            } elseif ($taskCase -eq 'omni') {
                $taskConf = $taskConf -replace '(?m)^#define CHASSIS_ENABLE_OMNIWHEEL .*', '#define CHASSIS_ENABLE_OMNIWHEEL 1'
            }
        }
        [IO.File]::WriteAllText((Join-Path $taskBuildDir 'Chassis_Conf.h'), $taskConf)
        $taskArgs = @('-std=c99','-Wall','-Wextra','-Werror','-Wconversion','-Wshadow','-Wstrict-prototypes','-pedantic',('-I'+$taskBuildDir))
        if ($Optimize) { $taskArgs += '-O2' }
        foreach ($taskInclude in @('APP/Inc','Device/Inc','BSP/Inc','Test/Inc','Chassis/Inc','algorithm/Inc')) {
            $taskArgs += '-I' + (Join-Path $taskRoot $taskInclude)
        }
        foreach ($taskSource in @('tests/test_chassis_app.c','APP/Src/ChassisApp.c','APP/Src/MotorApp.c',
            'Device/Src/DjiMotor.c','Chassis/Src/Chassis.c','Chassis/Src/Swerve.c',
            'algorithm/Src/PID_Config.c','algorithm/Src/PID.c','algorithm/Src/PositionLoop.c','algorithm/Src/Traj.c')) {
            $taskArgs += Join-Path $taskRoot $taskSource
        }
        & $Compiler @taskArgs -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw ('Compilation failed: ' + $taskCase) }
        & $taskExe
        if ($LASTEXITCODE -ne 0) { throw ('Tests failed: ' + $taskCase) }
    }
} finally {
    foreach ($taskName in @('main.h','fdcan.h','FreeRTOS.h','cmsis_os2.h','Chassis.h','Chassis_Conf.h','test_chassis_app.exe')) {
        $taskFile = Join-Path $taskBuildDir $taskName
        if (Test-Path -LiteralPath $taskFile) { Remove-Item -LiteralPath $taskFile }
    }
    Remove-Item -LiteralPath $taskBuildDir
}
