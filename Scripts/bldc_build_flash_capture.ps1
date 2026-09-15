# Build, program with J-Link, capture, and analyze one bounded BLDC test.
param(
    [string]$Port = "COM18",
    [int]$Baud = 460800,
    [ValidateSet("build", "rebuild")]
    [string]$BuildAction = "build",
    [string]$Uv4 = "",
    [string]$JLink = "",
    [string]$Device = "STM32F405RG",
    [int]$SwdSpeedKhz = 4000,
    [ValidateSet("auto", "open")]
    [string]$Mode = "auto",
    [ValidateSet("cw", "ccw")]
    [string]$Direction = "cw",
    [ValidateSet("up", "down")]
    [string]$SampleSide = "up",
    [double]$Duty = 0.08,
    [double]$Duration = 20.0,
    [double]$PeriodMs = 13.0,
    [Nullable[double]]$IntegralMvs = $null
)

$ErrorActionPreference = "Stop"
if ($Mode -eq "auto" -and ($Duty -lt 0.08 -or $Duty -gt 0.10)) {
    throw "Automatic BLDC duty must be in the verified range [0.08, 0.10]"
}
if ($Mode -eq "auto" -and $SampleSide -ne "up") {
    throw "Automatic BLDC mode requires SampleSide=up; down is fixed-open diagnostic-only"
}
if ($Mode -eq "open" -and ($Duty -le 0.0 -or $Duty -gt 0.25)) {
    throw "Fixed-open BLDC duty must be in (0, 0.25]"
}
if ($Mode -eq "auto" -and $null -ne $IntegralMvs -and
    ($IntegralMvs -lt 0.05 -or $IntegralMvs -gt 10.0)) {
    throw "Automatic BLDC integral threshold must be in [0.05, 10] mV*s"
}
$root = Split-Path $PSScriptRoot -Parent
$firmware = Join-Path (Split-Path $root -Parent) "build\foc.hex"
$runStamp = Get-Date -Format "yyyyMMdd-HHmmss"
$runDir = Join-Path $root ".build\bldc_pipeline\$runStamp"
$capturePrefix = Join-Path $root "Doc\verify\bldc_pipeline_$runStamp"
New-Item -ItemType Directory -Force -Path $runDir | Out-Null

function Resolve-JLinkPath {
    param([string]$Explicit)

    if ($Explicit) {
        if (!(Test-Path -LiteralPath $Explicit)) {
            throw "J-Link not found: $Explicit"
        }
        return (Resolve-Path -LiteralPath $Explicit).Path
    }
    foreach ($candidate in @(
            $env:JLINK_EXE,
            "D:\Users\15403\AppData\Local\Keil_v5\ARM\Segger\JLink.exe",
            "C:\Program Files\SEGGER\JLink\JLink.exe",
            "C:\Program Files (x86)\SEGGER\JLink\JLink.exe"
        )) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    throw "JLink.exe not found; pass -JLink or set JLINK_EXE"
}

Set-Location $root
Write-Host "[1/5] Stop motor output"
python Scripts\bldc_stop_serial.py --port $Port --baud $Baud
if ($LASTEXITCODE -ne 0) {
    throw "Unable to stop BLDC over $Port"
}

Write-Host "[2/5] Build firmware"
& (Join-Path $root "build_keil.ps1") -Project Foc -Action $BuildAction -Uv4 $Uv4
if ($LASTEXITCODE -ne 0) {
    throw "Keil build failed"
}
if (!(Test-Path -LiteralPath $firmware)) {
    throw "Firmware missing after build: $firmware"
}
$firmwareInfo = Get-Item -LiteralPath $firmware
$latestInput = Get-ChildItem -LiteralPath (Join-Path $root "App") -Recurse -File |
    Where-Object { $_.Extension -in @(".c", ".h", ".s", ".uvprojx") } |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1
if ($latestInput -and $firmwareInfo.LastWriteTime -lt $latestInput.LastWriteTime) {
    throw "Firmware is older than source: $($latestInput.FullName)"
}
Write-Host "Firmware: $firmware ($($firmwareInfo.Length) bytes)"

Write-Host "[3/5] Program and reset target"
$jlinkPath = Resolve-JLinkPath -Explicit $JLink
$jlinkScript = Join-Path $runDir "flash.jlink"
$jlinkLog = Join-Path $runDir "jlink.log"
$jlinkCommands = @(
    "r",
    "h",
    "loadfile $firmware",
    "r",
    "g",
    "q"
)
Set-Content -LiteralPath $jlinkScript -Value $jlinkCommands -Encoding ASCII
& $jlinkPath -device $Device -if SWD -speed $SwdSpeedKhz -autoconnect 1 `
    -CommanderScript $jlinkScript 2>&1 | Tee-Object -FilePath $jlinkLog
if ($LASTEXITCODE -ne 0) {
    throw "J-Link failed with exit code $LASTEXITCODE; see $jlinkLog"
}
$flashText = Get-Content -LiteralPath $jlinkLog -Raw
if ($flashText -notmatch "O\.K\." -or $flashText -match "(?im)^Error:") {
    throw "J-Link did not report a clean programming result; see $jlinkLog"
}

Write-Host "[4/5] Wait for reboot and capture bounded test"
Start-Sleep -Seconds 3
$captureArgs = @(
    "Scripts\bldc_capture_serial.py",
    "--port", $Port,
    "--baud", "$Baud",
    "--mode", $Mode,
    "--duty", "$Duty",
    "--direction", $Direction,
    "--sample-side", $SampleSide,
    "--duration", "$Duration",
    "--output", $capturePrefix
)
if ($Mode -eq "open") {
    $captureArgs += @("--period-ms", "$PeriodMs")
} elseif ($null -ne $IntegralMvs) {
    $captureArgs += @("--period-ms", "$PeriodMs", "--integral-mvs", "$IntegralMvs")
}
python @captureArgs
if ($LASTEXITCODE -ne 0) {
    throw "BLDC capture failed"
}

Write-Host "[5/5] Analyze captured status"
$csvPath = "$capturePrefix.csv"
if ($Mode -eq "open") {
    python Scripts\bldc_analyze_capture.py $csvPath --mode open --period-ms $PeriodMs
} else {
    python Scripts\bldc_analyze_capture.py $csvPath --mode auto
}
$analysisCode = $LASTEXITCODE
Write-Host "Log: $capturePrefix.log"
Write-Host "CSV: $csvPath"
if ($analysisCode -eq 2) {
    throw "BLDC analysis detected a failed run"
}
if ($analysisCode -eq 1) {
    Write-Warning "Run completed, but startup or commutation still has a nonfatal tuning issue."
}
