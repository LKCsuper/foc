param([string]$Compiler = "")
$ErrorActionPreference = "Stop"
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $root
try {
    New-Item -ItemType Directory -Force .build/host | Out-Null
    $motorInc = @('-IApp/motor/inc', '-IApp/motor/core/inc', '-IApp/motor/algo/inc', '-IApp/motor/loop/inc',
        '-IApp/motor/protect/inc', '-IApp/motor/start/inc', '-IApp/motor/observer/inc', '-IApp/motor/bldc/inc',
        '-IApp/motor/joint/inc', '-IApp/motor/calibration/inc', '-IApp/motor/diagnostic/inc')
    $timingSources = @('App/motor/diagnostic/motor_timing.c',
        'App/motor/diagnostic/motor_diag_handoff.c', 'App/motor/diagnostic/motor_diag_schedule.c',
        'App/motor/diagnostic/motor_diag_trace.c')
    & (Join-Path $PSScriptRoot 'test_isr_stack_budget.ps1')
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $root '.build/zig-cache'
    if (!$Compiler) {
        $Compiler = Join-Path $root '.build/tools/ziglang/zig.exe'
        if (!(Test-Path $Compiler)) { throw 'Specify -Compiler path to zig.exe, or install ziglang into .build/tools.' }
    }
    $cmdSource = Get-Content App/cmd/cmd_openloop.c -Raw -Encoding utf8
    $diagSource = Get-Content App/cmd/cmd_motor_diag.c -Raw -Encoding utf8
    $trigBench = [regex]::Match($diagSource, '(?s)static int ol_trig_bench\(void\)\s*\{.*?\r?\n\}')
    if (!$trigBench.Success) { throw 'Stopped trig benchmark missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/trig_bench_under_test.inc'), $trigBench.Value, [Text.UTF8Encoding]::new($false))
    $currentBench = [regex]::Match($diagSource, '(?s)static int ol_current_bench\(void\)\s*\{.*?\r?\n\}')
    if (!$currentBench.Success) { throw 'Stopped current benchmark missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/current_bench_under_test.inc'), $currentBench.Value, [Text.UTF8Encoding]::new($false))
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -DMOTOR_CFG_REFERENCE_TRIG=1 -ILibrary/Common/inc -I.build/host -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc Tests/host/test_current_bench.c -o .build/host/test_current_bench.exe
    if ($LASTEXITCODE -ne 0) { throw 'Current benchmark compilation failed' }
    & .build/host/test_current_bench.exe
    if ($LASTEXITCODE -ne 0) { throw 'Current benchmark tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -DMOTOR_CFG_REFERENCE_TRIG=1 @motorInc App/motor/diagnostic/motor_reference_trig.c Tests/host/test_reference_trig.c -o .build/host/test_reference_trig.exe
    if ($LASTEXITCODE -ne 0) { throw 'Reference trig compilation failed' }
    & .build/host/test_reference_trig.exe
    if ($LASTEXITCODE -ne 0) { throw 'Reference trig tests failed' }
    $rpmBrief = [regex]::Match($cmdSource, '(?s)static int ol_print_rpm_brief\(void\)\s*\{.*?\r?\n\}')
    if (!$rpmBrief.Success) { throw 'Brief RPM command missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/rpm_brief_under_test.inc'), $rpmBrief.Value, [Text.UTF8Encoding]::new($false))
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -I.build/host Tests/host/test_rpm_brief.c -o .build/host/test_rpm_brief.exe
    if ($LASTEXITCODE -ne 0) { throw 'Brief RPM command compilation failed' }
    & .build/host/test_rpm_brief.exe
    if ($LASTEXITCODE -ne 0) { throw 'Brief RPM command tests failed' }
    # 命令参数严格解析与 float 配置命令通用实现（截取 cmd_openloop.c 标记区间）。
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -IApp/cmd/inc Tests/host/test_cmd_args.c -o .build/host/test_cmd_args.exe
    if ($LASTEXITCODE -ne 0) { throw 'Command argument compilation failed' }
    & .build/host/test_cmd_args.exe
    if ($LASTEXITCODE -ne 0) { throw 'Command argument tests failed' }
    $cfgStart = $cmdSource.IndexOf('/* cmd config float:')
    $cfgEnd = $cmdSource.IndexOf('/* end cmd config float */', $cfgStart)
    if ($cfgStart -lt 0 -or $cfgEnd -le $cfgStart) { throw 'Config float command boundaries missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/cmd_config_float_under_test.inc'),
        $cmdSource.Substring($cfgStart, $cfgEnd - $cfgStart), [Text.UTF8Encoding]::new($false))
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -ILibrary/Common/inc -I.build/host -IApp/cmd/inc -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc Tests/host/test_cmd_config_float.c -o .build/host/test_cmd_config_float.exe
    if ($LASTEXITCODE -ne 0) { throw 'Config float command compilation failed' }
    & .build/host/test_cmd_config_float.exe
    if ($LASTEXITCODE -ne 0) { throw 'Config float command tests failed' }
    $irqSource = Get-Content Library/Hal/port/stm32f4/hal_motor_stm32f4.c -Raw -Encoding utf8
    $phaseSource = Get-Content App/motor/diagnostic/motor_diag_schedule.c -Raw -Encoding utf8
    $phaseCapture = [regex]::Match($phaseSource, '(?s)motor_phase_debug_t g_motor_phase_debug;.*?void Motor_Timing_PhaseEdgeNote\(.*?\r?\n\}')
    if (!$phaseCapture.Success) { throw 'Phase edge diagnostic missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/phase_edge_under_test.inc'), $phaseCapture.Value, [Text.UTF8Encoding]::new($false))
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -IApp/motor/diagnostic/inc -I.build/host Tests/host/test_phase_edge.c -o .build/host/test_phase_edge.exe
    if ($LASTEXITCODE -ne 0) { throw 'Phase edge compilation failed' }
    & .build/host/test_phase_edge.exe
    if ($LASTEXITCODE -ne 0) { throw 'Phase edge tests failed' }
    $irqEntry = Get-Content App/user/stm32f4xx_it.c -Raw -Encoding utf8
    $phaseIrq = [regex]::Match($irqEntry, '(?s)void DMA2_Stream4_IRQHandler\(void\)\s*\{.*?\r?\n\}')
    if (!$phaseIrq.Success) { throw 'Phase IRQ wrapper missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/phase_irq_under_test.inc'), $phaseIrq.Value, [Text.UTF8Encoding]::new($false))
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -I.build/host Tests/host/test_phase_irq.c -o .build/host/test_phase_irq.exe
    if ($LASTEXITCODE -ne 0) { throw 'Phase IRQ wrapper compilation failed' }
    & .build/host/test_phase_irq.exe
    if ($LASTEXITCODE -ne 0) { throw 'Phase IRQ wrapper tests failed' }
    $pwmWindow = [regex]::Match($irqSource, '(?s)hal_motor_pwm_window_t g_hal_motor_pwm_window;.*?void Hal_Motor_PwmSetDuty\(uint32_t duty_a, uint32_t duty_b, uint32_t duty_c\)\s*\{.*?\r?\n\}')
    if (!$pwmWindow.Success) { throw 'PWM window HAL missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/pwm_window_under_test.inc'), $pwmWindow.Value, [Text.UTF8Encoding]::new($false))
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -ILibrary/Hal/inc -I.build/host Tests/host/test_pwm_window.c -o .build/host/test_pwm_window.exe
    if ($LASTEXITCODE -ne 0) { throw 'PWM window HAL compilation failed' }
    & .build/host/test_pwm_window.exe
    if ($LASTEXITCODE -ne 0) { throw 'PWM window HAL tests failed' }
    $irqTest = [regex]::Match($irqSource, '(?s)static bool s_irq_test_active;.*?bool Hal_Motor_StoppedIrqTestEnd\(bool restore_gate\)\s*\{.*?\r?\n\}')
    if (!$irqTest.Success) { throw 'Stopped IRQ test HAL missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/stopped_irq_test_under_test.inc'), $irqTest.Value, [Text.UTF8Encoding]::new($false))
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -I.build/host Tests/host/test_stopped_irq_test.c -o .build/host/test_stopped_irq_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Stopped IRQ test HAL compilation failed' }
    & .build/host/test_stopped_irq_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Stopped IRQ test HAL tests failed' }
    $usartSource = Get-Content Library/Bsp/bsp_usart.c -Raw -Encoding utf8
    $usartWait = [regex]::Match($usartSource, '(?s)STATIC VOID Bsp_Usart_YieldToIsr\(VOID\)\s*\{.*?\r?\n\}')
    if (!$usartWait.Success) { throw 'USART wait function missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/usart_wait_under_test.inc'), $usartWait.Value, [Text.UTF8Encoding]::new($false))
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -I.build/host Tests/host/test_usart_wait.c -o .build/host/test_usart_wait.exe
    if ($LASTEXITCODE -ne 0) { throw 'USART wait compilation failed' }
    & .build/host/test_usart_wait.exe
    if ($LASTEXITCODE -ne 0) { throw 'USART wait tests failed' }
    $focSource = Get-Content App/motor/core/motor_foc_isr.c -Raw -Encoding utf8
    $statusPublish = [regex]::Match($focSource, '(?s)static void Motor_FocIsr_PublishStatus\(const MOTOR_PARAM_S \*motor_param\)\s*\{.*?\r?\n\}')
    $statusCounter = [regex]::Match($focSource, 'static uint\d+_t s_status_publish_divider;')
    $statusDivider = [regex]::Match($focSource, '#define MOTOR_STATUS_PUBLISH_DIVIDER \d+U')
    if (!$statusPublish.Success -or !$statusCounter.Success -or !$statusDivider.Success) { throw 'Status publish boundary missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/status_publish_under_test.inc'),
        ($statusDivider.Value + "`n" + $statusCounter.Value + "`n" + $statusPublish.Value), [Text.UTF8Encoding]::new($false))
    $focDispatch = [regex]::Match($focSource, '(?s)void Motor_FocIsr_Step\(MOTOR_PARAM_S \*motor_param\)\s*\{.*?\r?\n\}')
    if (!$focDispatch.Success) { throw 'FOC dispatch function missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/foc_dispatch_under_test.inc'), $focDispatch.Value, [Text.UTF8Encoding]::new($false))
    # 完整控制拍半周期门控（纯计算）
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror @motorInc App/motor/core/motor_foc_divider.c Tests/host/test_motor_foc_gate.c -o .build/host/test_motor_foc_gate.exe
    if ($LASTEXITCODE -ne 0) { throw 'FOC phase gate compilation failed' }
    & .build/host/test_motor_foc_gate.exe
    if ($LASTEXITCODE -ne 0) { throw 'FOC phase gate tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -ILibrary/Common/inc -IApp/motor/core/inc -I.build/host App/motor/core/motor_foc_divider.c Tests/host/test_foc_dispatch.c -o .build/host/test_foc_dispatch.exe
    if ($LASTEXITCODE -ne 0) { throw 'FOC dispatch compilation failed' }
    & .build/host/test_foc_dispatch.exe
    if ($LASTEXITCODE -ne 0) { throw 'FOC dispatch tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/algo/motor_math.c Tests/host/test_motor_math.c -o .build/host/test_motor_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor math compilation failed' }
    & .build/host/test_motor_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor math tests failed' }
    foreach ($assertMode in @('', '-DNDEBUG')) {
        $assertFlags = @()
        if ($assertMode) { $assertFlags += $assertMode }
        & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror @assertFlags Tests/host/test_common_assert.c -o .build/host/test_common_assert.exe
        if ($LASTEXITCODE -ne 0) { throw 'Common assertion compilation failed' }
        & .build/host/test_common_assert.exe
        if ($LASTEXITCODE -ne 0) { throw 'Common assertion tests failed' }
    }
    $halfSource = Get-Content App/motor/motor_it.c -Raw -Encoding utf8
    $halfStart = $halfSource.IndexOf('static void Motor_DmaInt_CaptureHalf(bool running)')
    $halfEnd = $halfSource.IndexOf('/**', $halfStart)
    if ($halfStart -lt 0 -or $halfEnd -le $halfStart) { throw 'Half callback capture boundary missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/half_capture_under_test.inc'),
        $halfSource.Substring($halfStart, $halfEnd - $halfStart), [Text.UTF8Encoding]::new($false))
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -I.build/host -IApp/motor_param/inc @motorInc Tests/host/test_motor_half_capture.c -o .build/host/test_motor_half_capture.exe
    if ($LASTEXITCODE -ne 0) { throw 'ADC half capture compilation failed' }
    & .build/host/test_motor_half_capture.exe
    if ($LASTEXITCODE -ne 0) { throw 'ADC half capture tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -ILibrary/Hal/inc @timingSources Tests/host/test_motor_timing_profile.c -o .build/host/test_motor_timing_profile.exe
    if ($LASTEXITCODE -ne 0) { throw 'Timing profile host compilation failed' }
    & .build/host/test_motor_timing_profile.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor timing profile host tests failed' }
    # 编译当前真实多速度磁链编排及拟合，测试拒绝条件和配置恢复。
    $fluxSource = Get-Content App/motor/calibration/motor_detect.c -Raw -Encoding utf8
    $refStart = $fluxSource.IndexOf('            if (s_flux_scan && (s_detect_control.samples % 16U) == 0U &&')
    $refEnd = $fluxSource.IndexOf("        if (s_detect_control.flux_coast_elapsed >=", $refStart)
    if ($refStart -lt 0 -or $refEnd -le $refStart) { throw 'Flux reference boundaries missing' }
    $refBody = $fluxSource.Substring($refStart, $refEnd - $refStart).TrimEnd()
    $refBody = $refBody.Substring(0, $refBody.LastIndexOf('}'))
    $rotStart = $fluxSource.IndexOf('        float minimum_run_duty = MOTOR_DETECT_FLUX_MIN_RUN_DUTY;')
    $rotEnd = $fluxSource.IndexOf('        if (s_detect_control.flux_duty_now < minimum_run_duty', $rotStart)
    if ($rotStart -lt 0 -or $rotEnd -le $rotStart) { throw 'Flux rotation boundaries missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/flux_rotation_under_test.inc'),
        "#define MOTOR_DETECT_FLUX_MIN_RUN_DUTY .03f`n#define MOTOR_DETECT_FLUX_DUTY_COLLAPSE_RATIO .70f`n" + $fluxSource.Substring($rotStart,$rotEnd-$rotStart),[Text.UTF8Encoding]::new($false))
    [IO.File]::WriteAllText((Join-Path $root '.build/host/flux_reference_under_test.inc'), $refBody, [Text.UTF8Encoding]::new($false))
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -I.build/host Tests/host/test_motor_flux_reference.c -o .build/host/test_motor_flux_reference.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flux reference compilation failed' }
    & .build/host/test_motor_flux_reference.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flux reference tests failed' }
    $fitStart = $fluxSource.IndexOf('static bool Detect_FitResistance(')
    $fitEnd = $fluxSource.IndexOf('static float Detect_PhaseCurrentPeak', $fitStart)
    $scanStart = $fluxSource.IndexOf('static bool Detect_MotorParametersInternal(')
    $scanEnd = $fluxSource.IndexOf('void DETECT_GetResult(', $scanStart)
    if ($fitStart -lt 0 -or $fitEnd -le $fitStart -or $scanStart -lt 0 -or $scanEnd -le $scanStart) { throw 'Flux scan boundaries missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/flux_scan_under_test.inc'),
        $fluxSource.Substring($fitStart, $fitEnd - $fitStart) + $fluxSource.Substring($scanStart, $scanEnd - $scanStart), [Text.UTF8Encoding]::new($false))
    $fluxConstants = ($fluxSource -split "`n" | Where-Object { $_.StartsWith('#define MOTOR_DETECT_') }) -join "`n"
    [IO.File]::WriteAllText((Join-Path $root '.build/host/flux_scan_constants.inc'), $fluxConstants, [Text.UTF8Encoding]::new($false))
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -I.build/host @motorInc -IApp/motor_param/inc Tests/host/test_motor_flux_scan.c -o .build/host/test_motor_flux_scan.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flux scan host compilation failed' }
    & .build/host/test_motor_flux_scan.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flux scan host tests failed' }
    # 编译当前真实命令片段，用主机替身验证任务等待、异常停机和配置恢复。
    $sltestSource = Get-Content App/cmd/cmd_sensorless.c -Raw -Encoding utf8
    $sltestStart = $sltestSource.IndexOf('/* sltest:')
    $sltestEnd = $sltestSource.IndexOf('/* end sltest */', $sltestStart)
    if ($sltestStart -lt 0 -or $sltestEnd -le $sltestStart) { throw 'sltest command boundaries missing' }
    [IO.File]::WriteAllText((Join-Path (Get-Location) '.build/host/sltest_under_test.inc'),
        $sltestSource.Substring($sltestStart, $sltestEnd - $sltestStart), [Text.UTF8Encoding]::new($false))
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -I.build/host @motorInc -IApp/motor_param/inc -IApp/motor_sensor/inc Tests/host/test_sensorless_speed_command.c -o .build/host/test_sensorless_speed_command.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensorless speed command host compilation failed' }
    & .build/host/test_sensorless_speed_command.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensorless speed command host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE "-Dassert_param(x)=((void)0)" @motorInc App/motor/algo/motor_foc.c App/motor/algo/motor_utils.c Tests/host/test_motor_fast_math.c -o .build/host/test_motor_fast_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Fast math host compilation failed' }
    & .build/host/test_motor_fast_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Fast math host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -IThirtyLibrary/common_code/lib/seqlock -IThirtyLibrary/common_code/lib/ringbuf -IThirtyLibrary/common_code/lib/textbuf ThirtyLibrary/common_code/lib/seqlock/seqlock.c ThirtyLibrary/common_code/lib/seqlock/seqlock_test.c ThirtyLibrary/common_code/lib/ringbuf/ringbuf.c ThirtyLibrary/common_code/lib/ringbuf/ringbuf_test.c ThirtyLibrary/common_code/lib/textbuf/textbuf.c ThirtyLibrary/common_code/lib/textbuf/textbuf_test.c Tests/host/test_common_code.c -o .build/host/test_common_code.exe
    if ($LASTEXITCODE -ne 0) { throw 'Common-code host compilation failed' }
    & .build/host/test_common_code.exe
    if ($LASTEXITCODE -ne 0) { throw 'Common-code host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -Wno-unused-variable -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -ILibrary/Common/inc App/motor_param/motor_settings.c App/motor_param/motor_param.c Tests/host/test_motor_settings.c -o .build/host/test_motor_settings.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
    & .build/host/test_motor_settings.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc Tests/host/test_motor_2804_profile.c -o .build/host/test_motor_2804_profile.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor 2804 profile host compilation failed' }
    & .build/host/test_motor_2804_profile.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor 2804 profile host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -Wno-unused-variable -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=3 -IApp/motor_param/inc @motorInc -ILibrary/Common/inc App/motor_param/motor_settings.c App/motor_param/motor_param.c Tests/host/test_motor_settings.c -o .build/host/test_motor_settings_2804.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor 2804 settings compilation failed' }
    & .build/host/test_motor_settings_2804.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor 2804 settings tests failed' }
    # Flash 线格式金标准：字段表改顺序/类型会在这里失败。
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -IApp/motor_param/inc @motorInc App/motor_param/motor_settings.c Tests/host/test_motor_settings_wire.c -o .build/host/test_motor_settings_wire.exe
    if ($LASTEXITCODE -ne 0) { throw 'Settings wire format compilation failed' }
    & .build/host/test_motor_settings_wire.exe
    if ($LASTEXITCODE -ne 0) { throw 'Settings wire format tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -Wno-int-to-void-pointer-cast -ITests/host/stubs -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor/loop/motor_anticogging.c App/motor_param/motor_settings.c App/motor_param/flash_storage.c Tests/host/stubs/hal_flash_host.c Tests/host/test_flash_storage.c -o .build/host/test_flash_storage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flash host compilation failed' }
    & .build/host/test_flash_storage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flash host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -IThirtyLibrary/common_code/lib/seqlock -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc -ILibrary/Hal/inc ThirtyLibrary/common_code/lib/seqlock/seqlock.c Tests/host/test_motor_snapshot.c -o .build/host/test_motor_snapshot.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor snapshot host compilation failed' }
    & .build/host/test_motor_snapshot.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor snapshot host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IThirtyLibrary/common_code/lib/seqlock -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc -ILibrary/Common/inc -ILibrary/Hal/inc ThirtyLibrary/common_code/lib/seqlock/seqlock.c App/motor/motor.c App/motor/loop/motor_anticogging.c App/motor/motor_snapshot.c App/motor_param/motor_settings.c App/motor_param/motor_param.c Tests/host/test_motor_api.c -o .build/host/test_motor_api.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor API host compilation failed' }
    & .build/host/test_motor_api.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor API host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc App/motor/loop/motor_speed_core.c App/motor/loop/motor_speed.c Tests/host/test_motor_speed.c -o .build/host/test_motor_speed.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed host compilation failed' }
    & .build/host/test_motor_speed.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/loop/motor_speed_ff_detect.c Tests/host/test_motor_speed_ff_detect.c -lm -o .build/host/test_motor_speed_ff_detect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed FF detect host compilation failed' }
    & .build/host/test_motor_speed_ff_detect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor speed FF detect host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/loop/motor_trajectory.c Tests/host/test_motor_trajectory.c -o .build/host/test_motor_trajectory.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor trajectory host compilation failed' }
    & .build/host/test_motor_trajectory.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor trajectory host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -ILibrary/Hal/inc @timingSources Tests/host/test_motor_timing.c -o .build/host/test_motor_timing.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor timing host compilation failed' }
    & .build/host/test_motor_timing.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor timing host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor/core/motor_output_core.c App/motor/core/motor_output.c Tests/host/test_motor_output.c -o .build/host/test_motor_output.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor output host compilation failed' }
    & .build/host/test_motor_output.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor output host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE "-Dassert_param(x)=((void)0)" @motorInc App/motor/algo/motor_foc.c App/motor/algo/motor_utils.c Tests/host/test_linear_svm_voltage.c -o .build/host/test_linear_svm_voltage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Linear SVM voltage compilation failed' }
    & .build/host/test_linear_svm_voltage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Linear SVM voltage tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc App/motor/diagnostic/motor_fast_vf.c Tests/host/test_motor_fast_vf.c -o .build/host/test_motor_fast_vf.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor fast VF host compilation failed' }
    & .build/host/test_motor_fast_vf.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor fast VF host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -ILibrary/Hal/inc App/motor/diagnostic/motor_fast_vf_isr.c Tests/host/test_motor_fast_vf_isr.c -o .build/host/test_motor_fast_vf_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor fast VF ISR host compilation failed' }
    & .build/host/test_motor_fast_vf_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor fast VF ISR host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -ILibrary/Hal/inc App/motor/bldc/motor_bldc_isr.c Tests/host/test_motor_bldc_isr.c -o .build/host/test_motor_bldc_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC ISR host compilation failed' }
    & .build/host/test_motor_bldc_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC ISR host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/bldc/motor_bldc_defaults.c Tests/host/test_motor_bldc_defaults.c -o .build/host/test_motor_bldc_defaults.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC defaults host compilation failed' }
    & .build/host/test_motor_bldc_defaults.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC defaults host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/bldc/motor_bldc_status.c Tests/host/test_motor_bldc_status.c -o .build/host/test_motor_bldc_status.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC status host compilation failed' }
    & .build/host/test_motor_bldc_status.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC status host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc App/motor_sensor/motor_sensor_update.c Tests/host/test_motor_sensor_update.c -o .build/host/test_motor_sensor_update.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensor update host compilation failed' }
    & .build/host/test_motor_sensor_update.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensor update host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=2 -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc App/motor/loop/motor_position_core.c App/motor/loop/motor_position_control.c App/motor/loop/motor_position.c Tests/host/test_motor_position.c -o .build/host/test_motor_position.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor position host compilation failed' }
    & .build/host/test_motor_position.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor position host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/joint/motor_joint_control.c Tests/host/test_motor_joint_control.c -o .build/host/test_motor_joint_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint control host compilation failed' }
    & .build/host/test_motor_joint_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint control host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/joint/motor_joint_protocol.c Tests/host/test_motor_joint_protocol.c -o .build/host/test_motor_joint_protocol.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint protocol host compilation failed' }
    & .build/host/test_motor_joint_protocol.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint protocol host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=2 -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor/loop/motor_trajectory.c App/motor/joint/motor_joint_control.c App/motor/joint/motor_joint.c Tests/host/test_motor_joint.c -o .build/host/test_motor_joint.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint host compilation failed' }
    & .build/host/test_motor_joint.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=3 -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor/loop/motor_trajectory.c App/motor/joint/motor_joint_control.c App/motor/joint/motor_joint.c Tests/host/test_motor_joint_trajectory.c -o .build/host/test_motor_joint_trajectory.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint trajectory host compilation failed' }
    & .build/host/test_motor_joint_trajectory.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor joint trajectory host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc App/motor/protect/motor_protect_core.c App/motor/protect/motor_protect.c Tests/host/stubs/protect_deps.c Tests/host/test_motor_protect.c -o .build/host/test_motor_protect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor protect host compilation failed' }
    & .build/host/test_motor_protect.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor protect host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/core/motor_foc_divider.c App/motor/start/motor_alignment.c App/motor/start/motor_if_start.c Tests/host/test_motor_startup.c -o .build/host/test_motor_startup.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor startup host compilation failed' }
    & .build/host/test_motor_startup.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor startup host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc App/motor/observer/motor_sensorless.c Tests/host/stubs/sensorless_math.c Tests/host/stubs/sensorless_deps.c Tests/host/test_motor_sensorless.c -o .build/host/test_motor_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensorless host compilation failed' }
    & .build/host/test_motor_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sensorless host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc App/motor/observer/motor_sensorless.c Tests/host/stubs/sensorless_math.c Tests/host/stubs/sensorless_deps.c Tests/host/test_sensorless_known_angle.c -o .build/host/test_sensorless_known_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Known-angle sensorless compilation failed' }
    & .build/host/test_sensorless_known_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Known-angle sensorless tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DSENSORLESS_REAL_MATH -IApp/motor_param/inc @motorInc App/motor/observer/motor_sensorless.c App/motor/algo/motor_math.c Tests/host/stubs/sensorless_math.c Tests/host/stubs/sensorless_deps.c Tests/host/test_sensorless_known_angle.c -o .build/host/test_sensorless_real_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Real-math sensorless compilation failed' }
    & .build/host/test_sensorless_real_math.exe
    if ($LASTEXITCODE -ne 0) { throw 'Real-math sensorless tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE '-Dassert_param(x)=((void)0)' -IApp/motor_param/inc @motorInc App/motor/observer/motor_sensorless.c App/motor/algo/motor_math.c App/motor/algo/motor_foc.c App/motor/algo/motor_utils.c Tests/host/stubs/sensorless_deps.c Tests/host/test_sensorless_disturbance.c -o .build/host/test_sensorless_disturbance.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensorless disturbance compilation failed' }
    & .build/host/test_sensorless_disturbance.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensorless disturbance tests failed' }
    & $Compiler cc -ILibrary/Common/inc -O2 -UNDEBUG -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE '-Dassert_param(x)=((void)0)' -IApp/motor_param/inc @motorInc App/motor/observer/motor_sensorless.c App/motor/algo/motor_math.c App/motor/algo/motor_foc.c App/motor/algo/motor_utils.c Tests/host/stubs/sensorless_deps.c Tests/host/test_sensorless_event_timing.c -o .build/host/test_sensorless_event_timing.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensorless event timing compilation failed' }
    & .build/host/test_sensorless_event_timing.exe
    if ($LASTEXITCODE -ne 0) { throw 'Sensorless event timing tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc App/motor/core/motor_current.c App/motor/algo/motor_pid.c Tests/host/stubs/current_math.c Tests/host/test_motor_current.c -o .build/host/test_motor_current.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor current host compilation failed' }
    & .build/host/test_motor_current.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor current host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/loop/motor_fw.c Tests/host/test_motor_fw.c -o .build/host/test_motor_fw.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor FW host compilation failed' }
    & .build/host/test_motor_fw.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor FW host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/algo/motor_utils.c App/motor/observer/motor_hfi.c Tests/host/test_motor_hfi.c -o .build/host/test_motor_hfi.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor HFI host compilation failed' }
    & .build/host/test_motor_hfi.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor HFI host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/algo/motor_utils.c App/motor/observer/motor_flux.c Tests/host/test_motor_flux.c -o .build/host/test_motor_flux.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor flux host compilation failed' }
    & .build/host/test_motor_flux.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor flux host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/algo/motor_utils.c App/motor/observer/motor_luenberger.c Tests/host/test_motor_luenberger.c -o .build/host/test_motor_luenberger.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor Luenberger host compilation failed' }
    & .build/host/test_motor_luenberger.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor Luenberger host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/algo/motor_utils.c App/motor/observer/motor_ekf.c Tests/host/test_motor_ekf.c -o .build/host/test_motor_ekf.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor EKF host compilation failed' }
    & .build/host/test_motor_ekf.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor EKF host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/algo/motor_utils.c App/motor/observer/motor_smo.c Tests/host/test_motor_smo.c -o .build/host/test_motor_smo.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor SMO host compilation failed' }
    & .build/host/test_motor_smo.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor SMO host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=2 -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc App/motor/algo/motor_utils.c App/motor/core/motor_output_core.c App/motor/core/motor_control.c App/motor/loop/motor_anticogging.c App/motor/loop/motor_fw.c App/motor/start/motor_alignment.c App/motor/start/motor_if_start.c App/motor/start/motor_hfi_start.c App/motor_sensor/motor_feedback.c Tests/host/test_motor_control.c -o .build/host/test_motor_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control host compilation failed' }
    & .build/host/test_motor_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_SELECTED_MODEL=3 -DMOTOR_CONTROL_HOLD_ONLY -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc App/motor/algo/motor_utils.c App/motor/core/motor_output_core.c App/motor/core/motor_control.c App/motor/loop/motor_anticogging.c App/motor/loop/motor_fw.c App/motor/start/motor_alignment.c App/motor/start/motor_if_start.c App/motor/start/motor_hfi_start.c App/motor_sensor/motor_feedback.c Tests/host/test_motor_control.c -o .build/host/test_motor_control_hold_2804.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control 2804 hold compilation failed' }
    & .build/host/test_motor_control_hold_2804.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor control 2804 hold tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/loop/motor_anticogging.c Tests/host/test_motor_anticogging.c -o .build/host/test_motor_anticogging.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor anticogging host compilation failed' }
    & .build/host/test_motor_anticogging.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor anticogging host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -ILibrary/Hal/inc App/motor/core/motor_sense.c Tests/host/test_motor_sense.c -o .build/host/test_motor_sense.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sense host compilation failed' }
    & .build/host/test_motor_sense.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor sense host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc App/motor/calibration/motor_detect_offset.c Tests/host/test_motor_detect_offset.c -o .build/host/test_motor_detect_offset.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect offset host compilation failed' }
    & .build/host/test_motor_detect_offset.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect offset host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -ILibrary/Hal/inc App/motor/calibration/motor_detect_isr.c Tests/host/test_motor_detect_isr.c -o .build/host/test_motor_detect_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect ISR host compilation failed' }
    & .build/host/test_motor_detect_isr.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor detect ISR host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_param/inc @motorInc -IApp/motor_sensor/inc App/motor_sensor/motor_feedback.c Tests/host/test_motor_feedback.c -o .build/host/test_motor_feedback.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor feedback host compilation failed' }
    & .build/host/test_motor_feedback.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor feedback host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -DMOTOR_MT6701_INTERFACE=1 -IApp/motor/inc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor_sensor/motor_mt6701_protocol.c App/motor_sensor/motor_mt6701.c Tests/host/test_motor_mt6701.c -o .build/host/test_motor_mt6701.exe
    if ($LASTEXITCODE -ne 0) { throw 'MT6701 host compilation failed' }
    & .build/host/test_motor_mt6701.exe
    if ($LASTEXITCODE -ne 0) { throw 'MT6701 host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -DMOTOR_MT6701_INTERFACE=2 -IApp/motor/inc -IApp/motor_sensor/inc -ILibrary/Hal/inc App/motor_sensor/motor_mt6701_protocol.c App/motor_sensor/motor_mt6701.c Tests/host/test_motor_mt6701_ssi.c -o .build/host/test_motor_mt6701_ssi.exe
    if ($LASTEXITCODE -ne 0) { throw 'MT6701 SSI host compilation failed' }
    & .build/host/test_motor_mt6701_ssi.exe
    if ($LASTEXITCODE -ne 0) { throw 'MT6701 SSI host tests failed' }
    # 仅核心+标准C，无HAL、motor参数或MT6701替身，证明可独立链接。
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -IApp/motor_sensor/inc App/motor_sensor/motor_encoder_core.c Tests/host/test_motor_encoder_core.c -o .build/host/test_motor_encoder_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Encoder standalone compilation failed' }
    & .build/host/test_motor_encoder_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Encoder standalone tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_ENCODER_HOST_TEST -IApp/motor_sensor/inc @motorInc -IApp/motor_param/inc -ILibrary/Hal/inc App/motor_sensor/motor_encoder_core.c App/motor_sensor/motor_encoder.c Tests/host/stubs/motor_mt6701_stub.c Tests/host/test_motor_encoder.c -o .build/host/test_motor_encoder.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor encoder host compilation failed' }
    & .build/host/test_motor_encoder.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor encoder host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -IApp/motor_sensor/inc @motorInc -IApp/motor_param/inc App/motor_sensor/motor_hall_angle.c Tests/host/test_motor_hall_angle.c -o .build/host/test_motor_hall_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor hall angle host compilation failed' }
    & .build/host/test_motor_hall_angle.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor hall angle host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/bldc/motor_bldc_six_step.c Tests/host/test_motor_bldc_six_step.c -o .build/host/test_motor_bldc_six_step.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC six-step host compilation failed' }
    & .build/host/test_motor_bldc_six_step.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC six-step host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE @motorInc App/motor/bldc/motor_bldc_six_step.c App/motor/bldc/motor_bldc_sensorless.c Tests/host/test_motor_bldc_sensorless.c -o .build/host/test_motor_bldc_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC sensorless host compilation failed' }
    & .build/host/test_motor_bldc_sensorless.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor BLDC sensorless host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DMOTOR_PHASE_ORDER_HOST_TEST -IApp/motor_sensor/inc App/motor_sensor/motor_phase_order.c Tests/host/test_motor_phase_order.c -o .build/host/test_motor_phase_order.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor phase order host compilation failed' }
    & .build/host/test_motor_phase_order.exe
    if ($LASTEXITCODE -ne 0) { throw 'Motor phase order host tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -D_DEFAULT_SOURCE "-Dassert_param(x)=((void)0)" @motorInc App/motor/algo/motor_foc.c App/motor/algo/motor_utils.c Examples/01_transform/main.c -o .build/host/example_01_transform.exe
    if ($LASTEXITCODE -ne 0) { throw 'Example 01 transform compilation failed' }
    & .build/host/example_01_transform.exe
    if ($LASTEXITCODE -ne 0) { throw 'Example 01 transform failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror @motorInc App/motor/loop/motor_speed_core.c Tests/host/test_motor_speed_core.c -o .build/host/test_motor_speed_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Speed standalone compilation failed' }
    & .build/host/test_motor_speed_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Speed standalone tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror @motorInc App/motor/protect/motor_protect_core.c App/motor/loop/motor_position_control.c Tests/host/test_motor_control_cores.c -o .build/host/test_motor_control_cores.exe
    if ($LASTEXITCODE -ne 0) { throw 'Protection/position standalone compilation failed' }
    & .build/host/test_motor_control_cores.exe
    if ($LASTEXITCODE -ne 0) { throw 'Protection/position standalone tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror @motorInc App/motor/loop/motor_position_core.c App/motor/loop/motor_position_control.c Tests/host/test_motor_position_core.c -o .build/host/test_motor_position_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Position mode standalone compilation failed' }
    & .build/host/test_motor_position_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Position mode standalone tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror @motorInc App/motor/core/motor_output_core.c Tests/host/test_motor_output_core.c -o .build/host/test_motor_output_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Output standalone compilation failed' }
    & .build/host/test_motor_output_core.exe
    if ($LASTEXITCODE -ne 0) { throw 'Output standalone tests failed' }
    & ./Scripts/export_reusable_modules.ps1
    $portableSources = @((Get-Content .build/reusable_modules/source_manifest.json -Raw | ConvertFrom-Json) | ForEach-Object {
        Join-Path '.build/reusable_modules/src' (Split-Path $_ -Leaf)
    })
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -I.build/reusable_modules/include @portableSources Tests/host/test_motor_encoder_core.c -o .build/host/test_reusable_modules.exe
    if ($LASTEXITCODE -ne 0) { throw 'Portable bundle compilation failed' }
    & .build/host/test_reusable_modules.exe
    if ($LASTEXITCODE -ne 0) { throw 'Portable bundle tests failed' }
    & $Compiler cc -ILibrary/Common/inc -std=c99 -Wall -Wextra -Werror -I.build/reusable_modules/include .build/reusable_modules/src/motor_mt6701_protocol.c Tests/host/test_motor_mt6701_protocol.c -o .build/host/test_motor_mt6701_protocol.exe
    if ($LASTEXITCODE -ne 0) { throw 'Standalone protocol compilation failed' }
    & .build/host/test_motor_mt6701_protocol.exe
    if ($LASTEXITCODE -ne 0) { throw 'Standalone protocol tests failed' }
} finally { Pop-Location }
