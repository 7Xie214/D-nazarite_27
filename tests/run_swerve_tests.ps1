param([string]$Compiler = 'gcc')

$ErrorActionPreference = 'Stop'
$taskProjectRoot = Split-Path -Parent $PSScriptRoot
$taskBuildDir = Join-Path ([System.IO.Path]::GetTempPath()) ('swerve-tests-' + [guid]::NewGuid())
New-Item -ItemType Directory -Path $taskBuildDir | Out-Null
$taskExe = Join-Path $taskBuildDir 'test_swerve.exe'

try {
    & $Compiler -std=c99 -Wall -Wextra -Werror -Wconversion -Wshadow -Wstrict-prototypes -pedantic `
        ('-I' + (Join-Path $taskProjectRoot 'Chassis/Inc')) `
        (Join-Path $taskProjectRoot 'tests/test_swerve.c') `
        (Join-Path $taskProjectRoot 'Chassis/Src/Swerve.c') -lm -o $taskExe
    if ($LASTEXITCODE -ne 0) { throw 'Swerve test compilation failed.' }
    & $taskExe
    if ($LASTEXITCODE -ne 0) { throw 'Swerve behavior tests failed.' }
} finally {
    # Only remove the explicitly named files and the now-empty temporary directory.
    if (Test-Path -LiteralPath $taskExe) { Remove-Item -LiteralPath $taskExe }
    Remove-Item -LiteralPath $taskBuildDir
}
