# 导出已经具备独立边界的算法及其直接头文件，不复制HAL、RTOS或全局电机配置。
param([string]$Destination = "")
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
if (!$Destination) { $Destination = Join-Path $taskRoot '.build/reusable_modules' }
$Destination = [IO.Path]::GetFullPath($Destination)
$taskSources = @(
    'App/motor_sensor/motor_encoder_core.c',
    'App/motor_sensor/motor_mt6701_protocol.c',
    'App/motor_sensor/motor_hall_angle.c',
    'App/motor/core/motor_output_core.c', 'App/motor/protect/motor_protect_core.c', 'App/motor/loop/motor_position_control.c', 'App/motor/loop/motor_position_core.c', 'App/motor/loop/motor_speed_core.c', 'App/motor/algo/motor_math.c', 'App/motor/algo/motor_utils.c', 'App/motor/algo/motor_foc.c', 'App/motor/algo/motor_pid.c',
    'App/motor/start/motor_alignment.c', 'App/motor/start/motor_if_start.c',
    'App/motor/start/motor_hfi_start.c',
    'App/motor/observer/motor_flux.c', 'App/motor/observer/motor_luenberger.c',
    'App/motor/observer/motor_ekf.c', 'App/motor/observer/motor_smo.c'
)
$taskInclude = Join-Path $Destination 'include'
$taskSrc = Join-Path $Destination 'src'
New-Item -ItemType Directory -Force $taskInclude, $taskSrc | Out-Null
$taskHeaders = @{}
$taskHeaders['common_assert.h'] = Join-Path $taskRoot 'Library/Common/inc/common_assert.h'
$taskHeaderDirs = @(Get-ChildItem (Join-Path $taskRoot 'App/motor') -Directory -Recurse -Filter 'inc') +
    @(Get-Item (Join-Path $taskRoot 'App/motor_sensor/inc'))
foreach ($taskDir in $taskHeaderDirs) {
    Get-ChildItem $taskDir.FullName -Filter '*.h' | ForEach-Object {
        if ($taskHeaders.ContainsKey($_.Name)) { throw "Duplicate header: $($_.Name)" }
        $taskHeaders[$_.Name] = $_.FullName
    }
}
$taskQueue = [Collections.Generic.Queue[string]]::new()
foreach ($taskSource in $taskSources) {
    $taskPath = Join-Path $taskRoot $taskSource
    Copy-Item -LiteralPath $taskPath -Destination $taskSrc
    $taskQueue.Enqueue($taskPath)
}
$taskCopied = @{}
while ($taskQueue.Count) {
    $taskPath = $taskQueue.Dequeue()
    $taskText = [IO.File]::ReadAllText($taskPath)
    foreach ($taskMatch in [regex]::Matches($taskText, '#include\s+"([^"]+)"')) {
        $taskName = $taskMatch.Groups[1].Value
        if (!$taskHeaders.ContainsKey($taskName)) { throw "Non-portable dependency: $taskPath -> $taskName" }
        if (!$taskCopied.ContainsKey($taskName)) {
            $taskCopied[$taskName] = $true
            Copy-Item -LiteralPath $taskHeaders[$taskName] -Destination $taskInclude
            $taskQueue.Enqueue($taskHeaders[$taskName])
        }
    }
}
Copy-Item -LiteralPath (Join-Path $taskRoot 'Doc/doc/模块独立复用.md') -Destination (Join-Path $Destination 'README.md')
$taskSources | ConvertTo-Json | Set-Content (Join-Path $Destination 'source_manifest.json') -Encoding utf8
Write-Output "已导出 $($taskSources.Count) 个算法源文件、$($taskCopied.Count) 个头文件到 $Destination"
