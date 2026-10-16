$ErrorActionPreference = 'Stop'
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $root
try {
    New-Item -ItemType Directory -Force .build/host | Out-Null
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $root '.build/zig-cache'
    & .build/tools/ziglang/zig.exe cc -std=c99 -Wall -Wextra -Werror -IApp/motor_param/inc Tests/host/test_motor_detect_rl_fit.c -o .build/host/test_motor_detect_rl_fit.exe
    if ($LASTEXITCODE -ne 0) { throw 'RL fit compilation failed' }
    & .build/host/test_motor_detect_rl_fit.exe
    if ($LASTEXITCODE -ne 0) { throw 'RL fit test failed' }
} finally { Pop-Location }
