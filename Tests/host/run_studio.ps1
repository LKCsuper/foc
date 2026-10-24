param([string]$Compiler = "")
$ErrorActionPreference = 'Stop'
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $root
try {
    if (!$Compiler) { $Compiler = Join-Path $root '.build/tools/ziglang/zig.exe' }
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $root '.build/zig-cache'
    New-Item -ItemType Directory -Force .build/host | Out-Null
    $source = Get-Content App/cmd/cmd_studio.c -Raw -Encoding utf8
    $body = $source.Substring($source.IndexOf('#if MOTOR_CFG_CMD') + '#if MOTOR_CFG_CMD'.Length)
    $body = $body.Substring(0, $body.LastIndexOf('#endif'))
    [IO.File]::WriteAllText((Join-Path $root '.build/host/studio_service_under_test.inc'), $body, [Text.UTF8Encoding]::new($false))
    $uart = Get-Content Library/Bsp/bsp_usart.c -Raw -Encoding utf8
    $frame = [regex]::Match($uart, '(?s)bool Bsp_Usart_TryWriteFrame\(.*?\r?\n\}')
    if (!$frame.Success) { throw 'Studio UART entry missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/studio_uart_under_test.inc'), $frame.Value, [Text.UTF8Encoding]::new($false))
    $commandSource = Get-Content App/cmd/command.c -Raw -Encoding utf8
    $handler = [regex]::Match($commandSource, '(?s)static void cmd_handle_char\(char c\)\s*\{.*?\r?\n\}')
    if (!$handler.Success) { throw 'Studio RX entry missing' }
    [IO.File]::WriteAllText((Join-Path $root '.build/host/studio_rx_under_test.inc'), $handler.Value, [Text.UTF8Encoding]::new($false))
    $inc = @('-IApp/cmd/inc', '-IApp/cmd', '-I.build/host', '-IApp/motor/inc', '-IApp/motor/core/inc',
        '-IApp/motor/algo/inc','-IApp/motor/loop/inc','-IApp/motor/start/inc','-IApp/motor/protect/inc',
        '-IApp/motor/observer/inc','-IApp/motor/calibration/inc','-IApp/motor/diagnostic/inc',
        '-IApp/motor/bldc/inc','-IApp/motor/joint/inc','-IApp/motor_param/inc','-IApp/motor_sensor/inc','-ILibrary/Common/inc')
    foreach ($test in @('protocol', 'service', 'rx')) {
        & $Compiler cc -std=c99 -Wall -Wextra -Werror @inc App/cmd/studio_protocol.c "Tests/host/test_studio_$test.c" -o ".build/host/test_studio_$test.exe"
        if ($LASTEXITCODE -ne 0) { throw "Studio $test compilation failed" }
        & ".build/host/test_studio_$test.exe"
        if ($LASTEXITCODE -ne 0) { throw "Studio $test failed" }
    }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -I.build/host -IThirtyLibrary/common_code/lib/ringbuf ThirtyLibrary/common_code/lib/ringbuf/ringbuf.c Tests/host/test_studio_uart.c -o .build/host/test_studio_uart.exe
    if ($LASTEXITCODE -ne 0) { throw 'Studio UART compilation failed' }
    & .build/host/test_studio_uart.exe
    if ($LASTEXITCODE -ne 0) { throw 'Studio UART failed' }
} finally { Pop-Location }
