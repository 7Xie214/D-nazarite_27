param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskDir = Join-Path ([IO.Path]::GetTempPath()) ('can-recovery-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $taskDir | Out-Null
try {
    $taskExe = Join-Path $taskDir 'test_can_recovery.exe'
    $taskArgs = @('-std=c99','-Wall','-Wextra','-Werror','-Wno-attributes',
        '-DSTM32H723xx','-DUSE_HAL_DRIVER')
    foreach ($taskInclude in @('Core/Inc','BSP/Inc','Drivers/STM32H7xx_HAL_Driver/Inc',
        'Drivers/CMSIS/Include','Drivers/CMSIS/Device/ST/STM32H7xx/Include')) {
        $taskArgs += '-I' + (Join-Path $taskRoot $taskInclude)
    }
    & $Compiler @taskArgs '-x' 'c' (Join-Path $taskRoot 'tests/test_can_recovery.c') (Join-Path $taskRoot 'BSP/Src/My_CAN.C') -o $taskExe
    if ($LASTEXITCODE -ne 0) { throw 'CAN recovery test compilation failed.' }
    & $taskExe
    if ($LASTEXITCODE -ne 0) { throw 'CAN recovery tests failed.' }
} finally {
    Get-ChildItem -LiteralPath $taskDir -File | Remove-Item
    Remove-Item -LiteralPath $taskDir
}
