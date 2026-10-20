# 通过 µVision (UV4) 批编译 App/Boot Keil 工程。
# 用法（在仓库根目录）:
#   .\build_keil.ps1
#   .\build_keil.ps1 -Action rebuild
#   .\build_keil.ps1 -Project Boot -Action clean
#   .\build_keil.ps1 -Uv4 "D:\path\UV4\UV4.exe"
#   .\build_keil.ps1 -Variant min -Defines "MOTOR_CFG_DIAG=0","MOTOR_CFG_CMD=0"
#
# -Variant 用临时工程副本追加宏定义，输出到 ..\build\<Variant>\，不改 foc.uvprojx 和默认输出。
#
# 实现位于 Scripts/build_keil.ps1；根目录 build_keil.ps1 仅作转发入口。
# UV4 退出码: 0=成功 1=仅警告 2=错误 3=致命；默认 0/1 视为通过，-StrictWarnings 时警告也失败。

param(
    [ValidateSet("Foc", "Boot")]
    [string]$Project = "Foc",

    [ValidateSet("build", "rebuild", "clean")]
    [string]$Action = "build",

    [string]$Uv4 = "",

    [string]$Target = "",

    [switch]$StrictWarnings,

    [string]$Variant = "",

    [string[]]$Defines = @()
)

$ErrorActionPreference = "Stop"

$root = Split-Path $PSScriptRoot -Parent
Set-Location $root

function Resolve-Uv4Path {
    param([string]$Explicit)

    if ($Explicit) {
        if (!(Test-Path -LiteralPath $Explicit)) {
            throw "UV4 not found: $Explicit"
        }
        return (Resolve-Path -LiteralPath $Explicit).Path
    }

    if ($env:KEIL_UV4 -and (Test-Path -LiteralPath $env:KEIL_UV4)) {
        return (Resolve-Path -LiteralPath $env:KEIL_UV4).Path
    }

    $candidates = New-Object System.Collections.Generic.List[string]

    foreach ($key in @(
            "HKLM:\SOFTWARE\WOW6432Node\Keil\Products\MDK",
            "HKLM:\SOFTWARE\Keil\Products\MDK",
            "HKCU:\SOFTWARE\Keil\Products\MDK"
        )) {
        if (Test-Path $key) {
            $armPath = (Get-ItemProperty $key -ErrorAction SilentlyContinue).Path
            if ($armPath) {
                $keilRoot = Split-Path $armPath -Parent
                $candidates.Add((Join-Path $keilRoot "UV4\UV4.exe"))
            }
        }
    }

    foreach ($p in @(
            "D:\Users\15403\AppData\Local\Keil_v5\UV4\UV4.exe",
            "C:\Keil_v5\UV4\UV4.exe",
            "D:\Keil_v5\UV4\UV4.exe",
            "E:\software_qrs\keil_core\UV4\UV4.exe",
            (Join-Path ${env:ProgramFiles} "Keil_v5\UV4\UV4.exe"),
            (Join-Path ${env:ProgramFiles(x86)} "Keil_v5\UV4\UV4.exe")
        )) {
        if ($p) { $candidates.Add([string]$p) }
    }

    foreach ($c in $candidates) {
        if ($c -and (Test-Path -LiteralPath $c)) {
            return (Resolve-Path -LiteralPath $c).Path
        }
    }

    throw @"
未找到 UV4.exe。请安装 Keil MDK，或任选其一：
  .\build_keil.ps1 -Uv4 `"C:\path\to\UV4\UV4.exe`"
  设置环境变量 KEIL_UV4 指向 UV4.exe
"@
}

$projMap = @{
    Foc  = @{
        Uvprojx = Join-Path $root "App\Project\foc.uvprojx"
        DefaultTarget = "Foc"
        OutHint = (Join-Path (Split-Path $root -Parent) "build\foc.axf")
    }
    Boot = @{
        Uvprojx = Join-Path $root "Boot\Project\Boot.uvprojx"
        DefaultTarget = "Boot"
        OutHint = Join-Path $root "Boot\Project\Objects\Boot.axf"
    }
}

$info = $projMap[$Project]
if (!(Test-Path -LiteralPath $info.Uvprojx)) {
    throw "Project file missing: $($info.Uvprojx)"
}

if (!$Target) {
    $Target = $info.DefaultTarget
}

$uv4Path = Resolve-Uv4Path -Explicit $Uv4
$logDir = Join-Path $root ".build\keil"
New-Item -ItemType Directory -Force -Path $logDir | Out-Null
$logFile = Join-Path $logDir ("{0}_{1}.log" -f $Project.ToLowerInvariant(), $Action)
$variantProject = ""

if ($Defines.Count -gt 0 -and !$Variant) {
    throw "-Defines requires -Variant so variant objects do not mix with the default build"
}
if ($Variant) {
    if ($Project -ne "Foc") { throw "-Variant is only supported for the Foc project" }
    if ($Variant -notmatch '^[A-Za-z0-9_]+$') { throw "Variant name must be [A-Za-z0-9_]+" }
    $text = Get-Content -LiteralPath $info.Uvprojx -Raw -Encoding UTF8
    $defineMatch = [regex]::Match($text, '<Define>(USE_STDPERIPH_DRIVER[^<]*)</Define>')
    if (!$defineMatch.Success) { throw "C compiler <Define> not found in $($info.Uvprojx)" }
    $extra = ($Defines | ForEach-Object { $_.Trim() } | Where-Object { $_ }) -join ','
    $newDefine = if ($extra) { "$($defineMatch.Groups[1].Value),$extra" } else { $defineMatch.Groups[1].Value }
    $text = $text.Remove($defineMatch.Index, $defineMatch.Length).Insert($defineMatch.Index, "<Define>$newDefine</Define>")
    $outDir = "..\..\..\build\$Variant\"
    $text = $text -replace '<OutputDirectory>[^<]*</OutputDirectory>', "<OutputDirectory>$outDir</OutputDirectory>"
    $text = $text -replace '<ListingPath>[^<]*</ListingPath>', "<ListingPath>$outDir</ListingPath>"
    $variantProject = Join-Path (Split-Path $info.Uvprojx -Parent) ("foc_variant_{0}.uvprojx" -f $Variant)
    [IO.File]::WriteAllText($variantProject, $text, (New-Object Text.UTF8Encoding($false)))
    $info = @{
        Uvprojx = $variantProject
        DefaultTarget = $info.DefaultTarget
        OutHint = (Join-Path (Split-Path $root -Parent) "build\$Variant\foc.axf")
    }
    $logFile = Join-Path $logDir ("foc_{0}_{1}.log" -f $Variant, $Action)
    Write-Host "Variant: $Variant ($newDefine)"
}

$flag = switch ($Action) {
    "build" { "-b" }
    "rebuild" { "-r" }
    "clean" { "-c" }
}

Write-Host "UV4:     $uv4Path"
Write-Host "Project: $($info.Uvprojx)"
Write-Host "Target:  $Target"
Write-Host "Action:  $Action ($flag)"
Write-Host "Log:     $logFile"

# UV4 批模式；-j0 不弹 GUI（部分版本支持）
$argsList = @($flag, $info.Uvprojx, "-t", $Target, "-o", $logFile, "-j0")
$proc = Start-Process -FilePath $uv4Path -ArgumentList $argsList -Wait -PassThru -NoNewWindow
$code = $proc.ExitCode
if ($variantProject) {
    Get-ChildItem -LiteralPath (Split-Path $variantProject -Parent) -Filter ("foc_variant_{0}.*" -f $Variant) |
        Remove-Item -Force -ErrorAction SilentlyContinue
}
$logText = ""

if (Test-Path -LiteralPath $logFile) {
    $logText = Get-Content -LiteralPath $logFile -Raw
    Write-Host "----- log (tail) -----"
    Get-Content -LiteralPath $logFile -Tail 40
    Write-Host "----- end log -----"
} else {
    Write-Warning "Log file was not created: $logFile"
}

# 0 成功；1 仅警告；2+ 失败
$logFailed = $logText -match "(?im)^error\s+-" -or
             $logText -match "(?im)^Target not created\."
$ok = (($code -eq 0) -or ((-not $StrictWarnings) -and ($code -eq 1))) -and
      (-not $logFailed)
if ($ok) {
    if ($Project -eq "Foc" -and $Action -ne "clean") {
        $mapPath = [IO.Path]::ChangeExtension($info.OutHint, '.map')
        $graphPath = [IO.Path]::ChangeExtension($info.OutHint, '.htm')
        $map = Get-Content -LiteralPath $mapPath -Raw
        $graph = Get-Content -LiteralPath $graphPath -Raw
        $stack = [regex]::Match($map, '(?m)^\s*STACK\s+0x[0-9a-fA-F]+\s+Section\s+(\d+)\s+startup_stm32f40xx\.o')
        $handler = [regex]::Match($graph, '(?s)DMA2_Stream4_IRQHandler</STRONG>(.*?)(?=<P>|$)')
        $depth = [regex]::Match($handler.Value, 'Max Depth = (\d+)')
        if (!$stack.Success -or !$depth.Success) { throw 'Missing FOC ISR stack budget evidence in linker outputs' }
        $stackBytes = [int]$stack.Groups[1].Value
        $depthBytes = [int]$depth.Groups[1].Value
        if ($stackBytes -lt ($depthBytes + 512)) {
            throw "FOC main stack insufficient: $stackBytes bytes, static DMA depth $depthBytes plus 512-byte exception/nesting reserve required"
        }
        Write-Host "FOC main stack: $stackBytes bytes; static DMA depth $depthBytes; reserve $($stackBytes - $depthBytes) bytes (runtime watermark still required)."
    }
    if ($code -eq 1) {
        Write-Host "Keil finished with warnings (exit $code)."
    } else {
        Write-Host "Keil finished OK (exit $code)."
    }
    if ($Action -ne "clean" -and (Test-Path -LiteralPath $info.OutHint)) {
        Write-Host "Output: $($info.OutHint)"
    }
    exit 0
}

if ($logFailed -and $code -lt 2) {
    $code = 2
}
Write-Error "Keil failed (exit $code). See $logFile"
exit $code
