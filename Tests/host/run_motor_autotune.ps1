$ErrorActionPreference = 'Stop'
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $root
try {
    New-Item -ItemType Directory -Force .build/host | Out-Null
    $motorInc = @('-IApp/motor/inc', '-IApp/motor/core/inc', '-IApp/motor/algo/inc', '-IApp/motor/loop/inc', '-IApp/motor/protect/inc', '-IApp/motor/start/inc', '-IApp/motor/observer/inc', '-IApp/motor/bldc/inc', '-IApp/motor/joint/inc', '-IApp/motor/calibration/inc', '-IApp/motor/diagnostic/inc')
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $root '.build/zig-cache'
    & .build/tools/ziglang/zig.exe cc -std=c99 -Wall -Wextra -Werror @motorInc App/motor/calibration/motor_autotune.c App/motor/algo/motor_pid.c Tests/host/test_motor_autotune.c -o .build/host/test_motor_autotune.exe
    if ($LASTEXITCODE -ne 0) { throw 'Autotune compilation failed' }
    & .build/host/test_motor_autotune.exe
    if ($LASTEXITCODE -ne 0) { throw 'Autotune test failed' }
    & .build/tools/ziglang/zig.exe cc -std=c99 -Wall -Wextra -Werror @motorInc -IApp/motor_sensor/inc -IApp/motor_param/inc -ILibrary/Hal/inc -ILibrary/Common/inc App/motor/calibration/motor_autotune.c App/cmd/cmd_autotune.c Tests/host/test_cmd_autotune.c -o .build/host/test_cmd_autotune.exe
    if ($LASTEXITCODE -ne 0) { throw 'Autotune command compilation failed' }
    & .build/host/test_cmd_autotune.exe
    if ($LASTEXITCODE -ne 0) { throw 'Autotune command test failed' }
} finally { Pop-Location }
