param([string]$Compiler = "")
$ErrorActionPreference = "Stop"
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $root
try {
    New-Item -ItemType Directory -Force .build/host | Out-Null
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $root '.build/zig-cache'
    if (!$Compiler) {
        $Compiler = Join-Path $root '.build/tools/ziglang/zig.exe'
        if (!(Test-Path $Compiler)) { throw 'Specify -Compiler path to zig.exe, or install ziglang into .build/tools.' }
    }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -Wno-unused-variable -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Common/inc App/motor_param/motor_settings.c App/motor_param/motor_param.c Tests/host/test_motor_settings.c -o .build/host/test_motor_settings.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
    & .build/host/test_motor_settings.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -Wno-int-to-void-pointer-cast -ITests/host/stubs -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor_param/motor_settings.c App/motor_param/flash_storage.c Tests/host/stubs/hal_flash_host.c Tests/host/test_flash_storage.c -o .build/host/test_flash_storage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flash host compilation failed' }
    & .build/host/test_flash_storage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flash host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc -ILibrary/Common/inc -ILibrary/Hal/inc App/motor/motor.c App/motor_param/motor_settings.c App/motor_param/motor_param.c Tests/host/test_motor_api.c -o .build/host/test_motor_api.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor API host compilation failed' }
    & .build/host/test_motor_api.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor API host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/motor_speed.c Tests/host/test_motor_speed.c -o .build/host/test_motor_speed.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed host compilation failed' }
    & .build/host/test_motor_speed.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/motor_protect.c Tests/host/stubs/protect_deps.c Tests/host/test_motor_protect.c -o .build/host/test_motor_protect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor protect host compilation failed' }
    & .build/host/test_motor_protect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor protect host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/motor_foc_divider.c App/motor/motor_alignment.c App/motor/motor_if_start.c Tests/host/test_motor_startup.c -o .build/host/test_motor_startup.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor startup host compilation failed' }
    & .build/host/test_motor_startup.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor startup host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/motor_sensorless.c Tests/host/stubs/sensorless_math.c Tests/host/stubs/sensorless_deps.c Tests/host/test_motor_sensorless.c -o .build/host/test_motor_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensorless host compilation failed' }
    & .build/host/test_motor_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensorless host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/motor_current.c App/motor/motor_pid.c Tests/host/stubs/current_math.c Tests/host/test_motor_current.c -o .build/host/test_motor_current.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor current host compilation failed' }
    & .build/host/test_motor_current.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor current host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc App/motor/motor_control.c App/motor/motor_alignment.c App/motor/motor_if_start.c App/motor/motor_hfi_start.c App/motor_sensor/motor_feedback.c Tests/host/test_motor_control.c -o .build/host/test_motor_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control host compilation failed' }
    & .build/host/test_motor_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Hal/inc App/motor/motor_sense.c Tests/host/test_motor_sense.c -o .build/host/test_motor_sense.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sense host compilation failed' }
    & .build/host/test_motor_sense.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sense host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor_param/motor_detect_offset.c Tests/host/test_motor_detect_offset.c -o .build/host/test_motor_detect_offset.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect offset host compilation failed' }
    & .build/host/test_motor_detect_offset.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect offset host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc App/motor_sensor/motor_feedback.c Tests/host/test_motor_feedback.c -o .build/host/test_motor_feedback.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor feedback host compilation failed' }
    & .build/host/test_motor_feedback.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor feedback host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_sensor/inc -IApp/motor/inc -IApp/motor_param/inc -ILibrary/Hal/inc App/motor_sensor/motor_encoder.c Tests/host/test_motor_encoder.c -o .build/host/test_motor_encoder.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor encoder host compilation failed' }
    & .build/host/test_motor_encoder.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor encoder host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_sensor/inc -IApp/motor/inc -IApp/motor_param/inc App/motor_sensor/motor_hall_angle.c Tests/host/test_motor_hall_angle.c -o .build/host/test_motor_hall_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor hall angle host compilation failed' }
    & .build/host/test_motor_hall_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor hall angle host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_PHASE_ORDER_HOST_TEST -IApp/motor_sensor/inc App/motor_sensor/motor_phase_order.c Tests/host/test_motor_phase_order.c -o .build/host/test_motor_phase_order.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor phase order host compilation failed' }
    & .build/host/test_motor_phase_order.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor phase order host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE "-Dassert_param(x)=((void)0)" -IApp/motor/inc App/motor/motor_foc.c App/motor/motor_utils.c Examples/01_transform/main.c -o .build/host/example_01_transform.exe
    if ($LASTEXITCODE -ne 0) { throw 'Example 01 transform compilation failed' }
    & .build/host/example_01_transform.exe
    if ($LASTEXITCODE -ne 0) { throw 'Example 01 transform failed' }
} finally { Pop-Location }
