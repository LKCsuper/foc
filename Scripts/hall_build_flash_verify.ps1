# Build, program with J-Link, and run Hall closed-loop verification.
param(
    [string]$Port = "COM17",
    [int]$Baud = 460800,
    [ValidateSet("build", "rebuild")]
    [string]$BuildAction = "build",
    [string]$Uv4 = "",
    [string]$JLink = "",
    [string]$Device = "STM32F405RG",
    [int]$SwdSpeedKhz = 1000,
    [ValidateSet("up", "down", "start72", "brake", "reverse", "all")]
    [string]$Profile = "all",
    [ValidateSet("cw", "ccw")]
    [string]$Direction = "cw",
    [ValidateSet("150", "300", "500", "800", "1200", "1500", "1800", "2000", "2500", "3000")]
    [int]$MaxRpm = 800,
    [int]$Samples = 3
)

$ErrorActionPreference = "Stop"
if ($MaxRpm -notin @(150, 300, 500, 800, 1200, 1500, 1800, 2000, 2500, 3000)) {
    throw "MaxRpm must be 150, 300, 500, 800, 1200, 1500, 1800, 2000, 2500, or 3000"
}
if ($Samples -lt 1 -or $Samples -gt 10) {
    throw "Samples must be 1..10"
}
$root = Split-Path $PSScriptRoot -Parent
$firmware = Join-Path (Split-Path $root -Parent) "build\foc.hex"
$runStamp = Get-Date -Format "yyyyMMdd-HHmmss"
$runDir = Join-Path $root ".build\hall_pipeline\$runStamp"
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
Write-Host "[1/4] Stop motor output"
python -X utf8 Scripts\foc_stop_serial.py --port $Port --baud $Baud
if ($LASTEXITCODE -ne 0) {
    Write-Warning "Unable to stop motor over $Port; continuing to flash"
}

Write-Host "[2/4] Build firmware"
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

Write-Host "[3/4] Program and reset target"
$jlinkPath = Resolve-JLinkPath -Explicit $JLink
$jlinkScript = Join-Path $runDir "flash.jlink"
$jlinkLog = Join-Path $runDir "jlink.log"
$jlinkCommands = @(
    "connect",
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
if ($flashText -match "No matching RAMCode|Failed to download|Could not connect to the target|Unspecified error") {
    throw "J-Link programming failed; see $jlinkLog"
}
if ($flashText -notmatch "(?s)Downloading file.*?O\.K\.") {
    throw "J-Link did not confirm hex download; see $jlinkLog"
}

Write-Host "[4/4] Wait for reboot and run Hall verification"
Start-Sleep -Seconds 8
python -X utf8 Scripts\hall_verify_serial.py --port $Port --baud $Baud --profile $Profile `
    --direction $Direction --max-rpm $MaxRpm --samples $Samples
$verifyCode = $LASTEXITCODE
if ($verifyCode -ne 0) {
    throw "Hall verification failed with exit code $verifyCode"
}
Write-Host "Hall verification passed ($Profile)"
