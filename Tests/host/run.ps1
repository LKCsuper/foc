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
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Hal/inc App/motor/motor_timing.c Tests/host/test_motor_timing_profile.c -o .build/host/test_motor_timing_profile.exe
    if ($LASTEXITCODE -ne 0) { throw 'Timing profile host compilation failed' }
    & .build/host/test_motor_timing_profile.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor timing profile host tests failed' }
    # 编译当前真实命令片段，用主机替身验证任务等待、异常停机和配置恢复。
    $sltestSource = Get-Content App/cmd/cmd_openloop.c -Raw -Encoding utf8
    $sltestStart = $sltestSource.IndexOf('/* sltest:')
    $sltestEnd = $sltestSource.IndexOf('/* end sltest */', $sltestStart)
    if ($sltestStart -lt 0 -or $sltestEnd -le $sltestStart) { throw 'sltest command boundaries missing' }
    [IO.File]::WriteAllText((Join-Path (Get-Location) '.build/host/sltest_under_test.inc'),
        $sltestSource.Substring($sltestStart, $sltestEnd - $sltestStart), [Text.UTF8Encoding]::new($false))
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -I.build/host -IApp/motor/inc -IApp/motor_param/inc -IApp/motor_sensor/inc Tests/host/test_sensorless_speed_command.c -o .build/host/test_sensorless_speed_command.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensorless speed command host compilation failed' }
    & .build/host/test_sensorless_speed_command.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensorless speed command host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE "-Dassert_param(x)=((void)0)" -IApp/motor/inc App/motor/motor_foc.c App/motor/motor_utils.c Tests/host/test_motor_fast_math.c -o .build/host/test_motor_fast_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Fast math host compilation failed' }
    & .build/host/test_motor_fast_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Fast math host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -IThirtyLibrary/common_code/lib/seqlock -IThirtyLibrary/common_code/lib/ringbuf -IThirtyLibrary/common_code/lib/textbuf ThirtyLibrary/common_code/lib/seqlock/seqlock.c ThirtyLibrary/common_code/lib/seqlock/seqlock_test.c ThirtyLibrary/common_code/lib/ringbuf/ringbuf.c ThirtyLibrary/common_code/lib/ringbuf/ringbuf_test.c ThirtyLibrary/common_code/lib/textbuf/textbuf.c ThirtyLibrary/common_code/lib/textbuf/textbuf_test.c Tests/host/test_common_code.c -o .build/host/test_common_code.exe
    if ($LASTEXITCODE -ne 0) { throw 'Common-code host compilation failed' }
    & .build/host/test_common_code.exe
    if ($LASTEXITCODE -ne 0) { throw 'Common-code host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -Wno-unused-variable -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Common/inc App/motor_param/motor_settings.c App/motor_param/motor_param.c Tests/host/test_motor_settings.c -o .build/host/test_motor_settings.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
    & .build/host/test_motor_settings.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc Tests/host/test_motor_2804_profile.c -o .build/host/test_motor_2804_profile.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor 2804 profile host compilation failed' }
    & .build/host/test_motor_2804_profile.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor 2804 profile host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -Wno-unused-variable -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=3 -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Common/inc App/motor_param/motor_settings.c App/motor_param/motor_param.c Tests/host/test_motor_settings.c -o .build/host/test_motor_settings_2804.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor 2804 settings compilation failed' }
    & .build/host/test_motor_settings_2804.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor 2804 settings tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -Wno-int-to-void-pointer-cast -ITests/host/stubs -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor/motor_anticogging.c App/motor_param/motor_settings.c App/motor_param/flash_storage.c Tests/host/stubs/hal_flash_host.c Tests/host/test_flash_storage.c -o .build/host/test_flash_storage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flash host compilation failed' }
    & .build/host/test_flash_storage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flash host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IThirtyLibrary/common_code/lib/seqlock -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc -ILibrary/Common/inc -ILibrary/Hal/inc ThirtyLibrary/common_code/lib/seqlock/seqlock.c App/motor/motor.c App/motor/motor_anticogging.c App/motor/motor_snapshot.c App/motor_param/motor_settings.c App/motor_param/motor_param.c Tests/host/test_motor_api.c -o .build/host/test_motor_api.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor API host compilation failed' }
    & .build/host/test_motor_api.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor API host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/motor_speed.c Tests/host/test_motor_speed.c -o .build/host/test_motor_speed.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed host compilation failed' }
    & .build/host/test_motor_speed.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/motor_speed_ff_detect.c Tests/host/test_motor_speed_ff_detect.c -lm -o .build/host/test_motor_speed_ff_detect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed FF detect host compilation failed' }
    & .build/host/test_motor_speed_ff_detect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed FF detect host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/motor_trajectory.c Tests/host/test_motor_trajectory.c -o .build/host/test_motor_trajectory.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor trajectory host compilation failed' }
    & .build/host/test_motor_trajectory.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor trajectory host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Hal/inc App/motor/motor_timing.c Tests/host/test_motor_timing.c -o .build/host/test_motor_timing.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor timing host compilation failed' }
    & .build/host/test_motor_timing.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor timing host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor/motor_output.c Tests/host/test_motor_output.c -o .build/host/test_motor_output.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor output host compilation failed' }
    & .build/host/test_motor_output.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor output host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE "-Dassert_param(x)=((void)0)" -IApp/motor/inc App/motor/motor_foc.c App/motor/motor_utils.c Tests/host/test_linear_svm_voltage.c -o .build/host/test_linear_svm_voltage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Linear SVM voltage compilation failed' }
    & .build/host/test_linear_svm_voltage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Linear SVM voltage tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/diagnostic/motor_fast_vf.c Tests/host/test_motor_fast_vf.c -o .build/host/test_motor_fast_vf.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor fast VF host compilation failed' }
    & .build/host/test_motor_fast_vf.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor fast VF host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Hal/inc App/motor/diagnostic/motor_fast_vf_isr.c Tests/host/test_motor_fast_vf_isr.c -o .build/host/test_motor_fast_vf_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor fast VF ISR host compilation failed' }
    & .build/host/test_motor_fast_vf_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor fast VF ISR host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Hal/inc App/motor/bldc/motor_bldc_isr.c Tests/host/test_motor_bldc_isr.c -o .build/host/test_motor_bldc_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC ISR host compilation failed' }
    & .build/host/test_motor_bldc_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC ISR host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/bldc/motor_bldc_defaults.c Tests/host/test_motor_bldc_defaults.c -o .build/host/test_motor_bldc_defaults.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC defaults host compilation failed' }
    & .build/host/test_motor_bldc_defaults.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC defaults host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/bldc/motor_bldc_status.c Tests/host/test_motor_bldc_status.c -o .build/host/test_motor_bldc_status.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC status host compilation failed' }
    & .build/host/test_motor_bldc_status.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC status host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc App/motor_sensor/motor_sensor_update.c Tests/host/test_motor_sensor_update.c -o .build/host/test_motor_sensor_update.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensor update host compilation failed' }
    & .build/host/test_motor_sensor_update.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensor update host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=2 -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc App/motor/motor_position_control.c App/motor/motor_position.c Tests/host/test_motor_position.c -o .build/host/test_motor_position.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor position host compilation failed' }
    & .build/host/test_motor_position.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor position host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/joint/motor_joint_control.c Tests/host/test_motor_joint_control.c -o .build/host/test_motor_joint_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint control host compilation failed' }
    & .build/host/test_motor_joint_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint control host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/joint/motor_joint_protocol.c Tests/host/test_motor_joint_protocol.c -o .build/host/test_motor_joint_protocol.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint protocol host compilation failed' }
    & .build/host/test_motor_joint_protocol.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint protocol host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=2 -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor/motor_trajectory.c App/motor/joint/motor_joint_control.c App/motor/joint/motor_joint.c Tests/host/test_motor_joint.c -o .build/host/test_motor_joint.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint host compilation failed' }
    & .build/host/test_motor_joint.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=3 -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor/motor_trajectory.c App/motor/joint/motor_joint_control.c App/motor/joint/motor_joint.c Tests/host/test_motor_joint_trajectory.c -o .build/host/test_motor_joint_trajectory.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint trajectory host compilation failed' }
    & .build/host/test_motor_joint_trajectory.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint trajectory host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/motor_protect.c Tests/host/stubs/protect_deps.c Tests/host/test_motor_protect.c -o .build/host/test_motor_protect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor protect host compilation failed' }
    & .build/host/test_motor_protect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor protect host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/motor_foc_divider.c App/motor/start/motor_alignment.c App/motor/start/motor_if_start.c Tests/host/test_motor_startup.c -o .build/host/test_motor_startup.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor startup host compilation failed' }
    & .build/host/test_motor_startup.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor startup host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/observer/motor_sensorless.c Tests/host/stubs/sensorless_math.c Tests/host/stubs/sensorless_deps.c Tests/host/test_motor_sensorless.c -o .build/host/test_motor_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensorless host compilation failed' }
    & .build/host/test_motor_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensorless host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/observer/motor_sensorless.c Tests/host/stubs/sensorless_math.c Tests/host/stubs/sensorless_deps.c Tests/host/test_sensorless_known_angle.c -o .build/host/test_sensorless_known_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Known-angle sensorless compilation failed' }
    & .build/host/test_sensorless_known_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Known-angle sensorless tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DSENSORLESS_REAL_MATH -IApp/motor_param/inc -IApp/motor/inc App/motor/observer/motor_sensorless.c App/motor/motor_math.c Tests/host/stubs/sensorless_math.c Tests/host/stubs/sensorless_deps.c Tests/host/test_sensorless_known_angle.c -o .build/host/test_sensorless_real_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Real-math sensorless compilation failed' }
    & .build/host/test_sensorless_real_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Real-math sensorless tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor/motor_current.c App/motor/motor_pid.c Tests/host/stubs/current_math.c Tests/host/test_motor_current.c -o .build/host/test_motor_current.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor current host compilation failed' }
    & .build/host/test_motor_current.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor current host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/motor_fw.c Tests/host/test_motor_fw.c -o .build/host/test_motor_fw.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor FW host compilation failed' }
    & .build/host/test_motor_fw.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor FW host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/observer/motor_hfi.c Tests/host/test_motor_hfi.c -o .build/host/test_motor_hfi.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor HFI host compilation failed' }
    & .build/host/test_motor_hfi.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor HFI host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/observer/motor_flux.c Tests/host/test_motor_flux.c -o .build/host/test_motor_flux.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor flux host compilation failed' }
    & .build/host/test_motor_flux.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor flux host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/observer/motor_luenberger.c Tests/host/test_motor_luenberger.c -o .build/host/test_motor_luenberger.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor Luenberger host compilation failed' }
    & .build/host/test_motor_luenberger.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor Luenberger host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/observer/motor_ekf.c Tests/host/test_motor_ekf.c -o .build/host/test_motor_ekf.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor EKF host compilation failed' }
    & .build/host/test_motor_ekf.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor EKF host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/observer/motor_smo.c Tests/host/test_motor_smo.c -o .build/host/test_motor_smo.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor SMO host compilation failed' }
    & .build/host/test_motor_smo.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor SMO host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=2 -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc App/motor/motor_control.c App/motor/motor_anticogging.c App/motor/motor_fw.c App/motor/start/motor_alignment.c App/motor/start/motor_if_start.c App/motor/start/motor_hfi_start.c App/motor_sensor/motor_feedback.c Tests/host/test_motor_control.c -o .build/host/test_motor_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control host compilation failed' }
    & .build/host/test_motor_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=3 -DMOTOR_CONTROL_HOLD_ONLY -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc App/motor/motor_control.c App/motor/motor_anticogging.c App/motor/motor_fw.c App/motor/start/motor_alignment.c App/motor/start/motor_if_start.c App/motor/start/motor_hfi_start.c App/motor_sensor/motor_feedback.c Tests/host/test_motor_control.c -o .build/host/test_motor_control_hold_2804.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control 2804 hold compilation failed' }
    & .build/host/test_motor_control_hold_2804.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control 2804 hold tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/motor_anticogging.c Tests/host/test_motor_anticogging.c -o .build/host/test_motor_anticogging.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor anticogging host compilation failed' }
    & .build/host/test_motor_anticogging.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor anticogging host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Hal/inc App/motor/motor_sense.c Tests/host/test_motor_sense.c -o .build/host/test_motor_sense.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sense host compilation failed' }
    & .build/host/test_motor_sense.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sense host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc App/motor_param/motor_detect_offset.c Tests/host/test_motor_detect_offset.c -o .build/host/test_motor_detect_offset.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect offset host compilation failed' }
    & .build/host/test_motor_detect_offset.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect offset host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Hal/inc App/motor/calibration/motor_detect_isr.c Tests/host/test_motor_detect_isr.c -o .build/host/test_motor_detect_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect ISR host compilation failed' }
    & .build/host/test_motor_detect_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect ISR host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -IApp/motor_sensor/inc App/motor_sensor/motor_feedback.c Tests/host/test_motor_feedback.c -o .build/host/test_motor_feedback.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor feedback host compilation failed' }
    & .build/host/test_motor_feedback.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor feedback host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -DMOTOR_MT6701_INTERFACE=1 -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor_sensor/motor_mt6701.c Tests/host/test_motor_mt6701.c -o .build/host/test_motor_mt6701.exe
    if ($LASTEXITCODE -ne 0) { throw 'MT6701 host compilation failed' }
    & .build/host/test_motor_mt6701.exe
    if ($LASTEXITCODE -ne 0) { throw 'MT6701 host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -DMOTOR_MT6701_INTERFACE=2 -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor_sensor/motor_mt6701.c Tests/host/test_motor_mt6701_ssi.c -o .build/host/test_motor_mt6701_ssi.exe
    if ($LASTEXITCODE -ne 0) { throw 'MT6701 SSI host compilation failed' }
    & .build/host/test_motor_mt6701_ssi.exe
    if ($LASTEXITCODE -ne 0) { throw 'MT6701 SSI host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_ENCODER_HOST_TEST -IApp/motor_sensor/inc -IApp/motor/inc -IApp/motor_param/inc -ILibrary/Hal/inc App/motor_sensor/motor_encoder.c Tests/host/stubs/motor_mt6701_stub.c Tests/host/test_motor_encoder.c -o .build/host/test_motor_encoder.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor encoder host compilation failed' }
    & .build/host/test_motor_encoder.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor encoder host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_sensor/inc -IApp/motor/inc -IApp/motor_param/inc App/motor_sensor/motor_hall_angle.c Tests/host/test_motor_hall_angle.c -o .build/host/test_motor_hall_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor hall angle host compilation failed' }
    & .build/host/test_motor_hall_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor hall angle host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/bldc/motor_bldc_six_step.c Tests/host/test_motor_bldc_six_step.c -o .build/host/test_motor_bldc_six_step.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC six-step host compilation failed' }
    & .build/host/test_motor_bldc_six_step.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC six-step host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor/inc App/motor/bldc/motor_bldc_six_step.c App/motor/bldc/motor_bldc_sensorless.c Tests/host/test_motor_bldc_sensorless.c -o .build/host/test_motor_bldc_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC sensorless host compilation failed' }
    & .build/host/test_motor_bldc_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC sensorless host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_PHASE_ORDER_HOST_TEST -IApp/motor_sensor/inc App/motor_sensor/motor_phase_order.c Tests/host/test_motor_phase_order.c -o .build/host/test_motor_phase_order.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor phase order host compilation failed' }
    & .build/host/test_motor_phase_order.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor phase order host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE "-Dassert_param(x)=((void)0)" -IApp/motor/inc App/motor/motor_foc.c App/motor/motor_utils.c Examples/01_transform/main.c -o .build/host/example_01_transform.exe
    if ($LASTEXITCODE -ne 0) { throw 'Example 01 transform compilation failed' }
    & .build/host/example_01_transform.exe
    if ($LASTEXITCODE -ne 0) { throw 'Example 01 transform failed' }
} finally { Pop-Location }
