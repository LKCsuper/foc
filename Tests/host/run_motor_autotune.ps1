$ErrorActionPreference = 'Stop'
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $root
try {
    New-Item -ItemType Directory -Force .build/host | Out-Null
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $root '.build/zig-cache'
    & .build/tools/ziglang/zig.exe cc -std=c99 -Wall -Wextra -Werror -IApp/motor/inc App/motor/motor_autotune.c App/motor/motor_pid.c Tests/host/test_motor_autotune.c -o .build/host/test_motor_autotune.exe
    if ($LASTEXITCODE -ne 0) { throw 'Autotune compilation failed' }
    & .build/host/test_motor_autotune.exe
    if ($LASTEXITCODE -ne 0) { throw 'Autotune test failed' }
    & .build/tools/ziglang/zig.exe cc -std=c99 -Wall -Wextra -Werror -IApp/motor/inc -IApp/motor_sensor/inc -IApp/motor_param/inc -ILibrary/Hal/inc -ILibrary/Common/inc App/motor/motor_autotune.c App/cmd/cmd_autotune.c Tests/host/test_cmd_autotune.c -o .build/host/test_cmd_autotune.exe
    if ($LASTEXITCODE -ne 0) { throw 'Autotune command compilation failed' }
    & .build/host/test_cmd_autotune.exe
    if ($LASTEXITCODE -ne 0) { throw 'Autotune command test failed' }
} finally { Pop-Location }
