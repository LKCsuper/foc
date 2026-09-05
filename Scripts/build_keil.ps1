# 通过 µVision (UV4) 批编译 App/Boot Keil 工程。
# 用法（在仓库根目录）:
#   .\build_keil.ps1
#   .\build_keil.ps1 -Action rebuild
#   .\build_keil.ps1 -Project Boot -Action clean
#   .\build_keil.ps1 -Uv4 "D:\path\UV4\UV4.exe"
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

    [switch]$StrictWarnings
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

if (Test-Path -LiteralPath $logFile) {
    Write-Host "----- log (tail) -----"
    Get-Content -LiteralPath $logFile -Tail 40
    Write-Host "----- end log -----"
} else {
    Write-Warning "Log file was not created: $logFile"
}

# 0 成功；1 仅警告；2+ 失败
$ok = ($code -eq 0) -or ((-not $StrictWarnings) -and ($code -eq 1))
if ($ok) {
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

Write-Error "Keil failed (exit $code). See $logFile"
exit $code
