param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$taskProjectRoot = Split-Path -Parent $PSScriptRoot
$taskBuildDir = Join-Path ([System.IO.Path]::GetTempPath()) ('chassis-init-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $taskBuildDir | Out-Null
$taskConfPath = Join-Path $taskBuildDir 'Chassis_Conf.h'
$taskHeaderPath = Join-Path $taskBuildDir 'Chassis.h'
$taskBaseConfig = Get-Content -LiteralPath (Join-Path $taskProjectRoot 'Chassis/Inc/Chassis_Conf.h') -Raw
$taskOutputFiles = @($taskConfPath, $taskHeaderPath)
$taskScenes = @(
    @{ Name = 'default'; Defines = @(); Reject = $false },
    @{ Name = 'override'; Defines = @(
        '-DCHASSIS_SWERVE_WHEEL_RADIUS_M=0.075f',
        '-DCHASSIS_SWERVE_HALF_WHEEL_BASE_M=0.3f',
        '-DCHASSIS_SWERVE_HALF_TRACK_WIDTH_M=0.2f',
        '-DCHASSIS_SWERVE_FL_ZERO_OFFSET=111u',
        '-DCHASSIS_SWERVE_FL_STEER_ENCODER_DIRECTION=-1',
        '-DCHASSIS_SWERVE_FL_DRIVE_ENCODER_DIRECTION=-1',
        '-DCHASSIS_SWERVE_FL_INITIAL_ANGLE_RAD=0.1f',
        '-DCHASSIS_SWERVE_FR_ZERO_OFFSET=222u',
        '-DCHASSIS_SWERVE_FR_STEER_ELECTRIC_DIRECTION=-1',
        '-DCHASSIS_SWERVE_FR_DRIVE_ELECTRIC_DIRECTION=-1',
        '-DCHASSIS_SWERVE_FR_ANGLE_RANGE=SWERVE_ANGLE_0_TO_180',
        '-DCHASSIS_SWERVE_FR_INITIAL_ANGLE_RAD=0.2f',
        '-DCHASSIS_SWERVE_RL_ZERO_OFFSET=333u',
        '-DCHASSIS_SWERVE_RL_STEER_ENCODER_DIRECTION=-1',
        '-DCHASSIS_SWERVE_RL_STEER_ELECTRIC_DIRECTION=-1',
        '-DCHASSIS_SWERVE_RL_ANGLE_RANGE=SWERVE_ANGLE_CONTINUOUS',
        '-DCHASSIS_SWERVE_RL_INITIAL_ANGLE_RAD=-4.0f',
        '-DCHASSIS_SWERVE_RR_ZERO_OFFSET=444u',
        '-DCHASSIS_SWERVE_RR_DRIVE_ENCODER_DIRECTION=-1',
        '-DCHASSIS_SWERVE_RR_DRIVE_ELECTRIC_DIRECTION=-1',
        '-DCHASSIS_SWERVE_RR_INITIAL_ANGLE_RAD=-0.3f'
    ); Reject = $false },
    @{ Name = 'disabled'; Defines = @('-DCHASSIS_ENABLE_SWERVE=0'); Reject = $false },
    @{ Name = 'mecanum'; Defines = @('-DCHASSIS_ENABLE_SWERVE=0', '-DCHASSIS_ENABLE_MECANUM=1'); Reject = $false },
    @{ Name = 'omni'; Defines = @('-DCHASSIS_ENABLE_SWERVE=0', '-DCHASSIS_ENABLE_OMNIWHEEL=1'); Reject = $false },
    @{ Name = 'conflict'; Defines = @('-DCHASSIS_ENABLE_MECANUM=1'); Reject = $true }
)

try {
    Copy-Item -LiteralPath (Join-Path $taskProjectRoot 'Chassis/Inc/Chassis.h') -Destination $taskHeaderPath
    foreach ($taskScene in $taskScenes) {
        # Change a temporary header just as a user edits the real configuration.
        $taskConfig = $taskBaseConfig
        foreach ($taskDefine in $taskScene.Defines) {
            $taskSetting = $taskDefine.Substring(2).Split('=', 2)
            $taskPattern = '(?m)^#define[ \t]+' + [regex]::Escape($taskSetting[0]) + '[ \t]+[^\r\n]+'
            if (-not [regex]::IsMatch($taskConfig, $taskPattern)) { throw ('Missing macro: ' + $taskSetting[0]) }
            $taskConfig = $taskConfig -replace $taskPattern, ('#define ' + $taskSetting[0] + ' ' + $taskSetting[1])
        }
        [System.IO.File]::WriteAllText($taskConfPath, $taskConfig, [System.Text.UTF8Encoding]::new($false))
        $taskExe = Join-Path $taskBuildDir ($taskScene.Name + '.exe')
        $taskOutputFiles += $taskExe
        $taskArgs = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-Wconversion',
            '-Wshadow', '-Wstrict-prototypes', '-pedantic',
            ('-I' + $taskBuildDir),
            ('-I' + (Join-Path $taskProjectRoot 'Chassis/Inc')),
            (Join-Path $taskProjectRoot 'tests/test_chassis_init.c'),
            (Join-Path $taskProjectRoot 'Chassis/Src/Chassis.c'), '-o', $taskExe)
        # Expected preprocessor failures write to stderr; capture them without
        # PowerShell treating those diagnostics as its own terminating errors.
        $ErrorActionPreference = 'Continue'
        $taskCompilerOutput = & $Compiler @taskArgs 2>&1
        $taskCompilerExit = $LASTEXITCODE
        $ErrorActionPreference = 'Stop'
        if ($taskScene.Reject) {
            if ($taskCompilerExit -eq 0) { throw ($taskScene.Name + ': invalid selection compiled successfully.') }
            if (($taskCompilerOutput | Out-String) -notmatch 'CHASSIS_ENABLE_') {
                throw ($taskScene.Name + ': compilation failed for an unexpected reason: ' + ($taskCompilerOutput | Out-String))
            }
            Write-Output ($taskScene.Name + ': rejected as expected')
        } else {
            if ($taskCompilerExit -ne 0) { throw ($taskCompilerOutput | Out-String) }
            Write-Output ($taskScene.Name + ':')
            & $taskExe
            if ($LASTEXITCODE -ne 0) { throw ($taskScene.Name + ': initialization checks failed.') }
        }
    }
    Write-Output 'All 6 chassis configuration build cases passed.'
} finally {
    $ErrorActionPreference = 'Stop'
    foreach ($taskOutputFile in $taskOutputFiles) {
        if (Test-Path -LiteralPath $taskOutputFile) { Remove-Item -LiteralPath $taskOutputFile }
    }
    Remove-Item -LiteralPath $taskBuildDir
}
