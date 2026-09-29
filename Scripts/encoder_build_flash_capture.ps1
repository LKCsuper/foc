# Build, program with J-Link at 100 kHz, and capture the encoder 150 RPM speed loop.
param(
    [string]$Port = "COM5",
    [int]$Baud = 460800,
    [ValidateSet("build", "rebuild")]
    [string]$BuildAction = "build",
    [string]$Uv4 = "",
    [string]$JLink = "",
    [string]$Device = "STM32F405RG",
    [int]$SwdSpeedKhz = 100,
    [int]$Samples = 8,
    [double]$SpeedKp = 0.0005,
    [double]$SpeedKi = 0.00015,
    [double]$SpeedIq = 0.08,
    [double]$CurrentKp = 1.06,
    [double]$CurrentKi = 2120
)

$ErrorActionPreference = "Stop"
if ($Samples -lt 1 -or $Samples -gt 12) {
    throw "Samples must be 1..12"
}
$root = Split-Path $PSScriptRoot -Parent
$firmware = Join-Path (Split-Path $root -Parent) "build\foc.hex"
$runStamp = Get-Date -Format "yyyyMMdd-HHmmss"
$runDir = Join-Path $root ".build\encoder_pipeline\$runStamp"
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
$latestInput = Get-ChildItem -LiteralPath (Join-Path $root "App"), (Join-Path $root "Library") -Recurse -File -ErrorAction SilentlyContinue |
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

Write-Host "[4/4] Wait for reboot and capture encoder speed loop"
Start-Sleep -Seconds 8
python -X utf8 Scripts\encoder_speed_capture.py --port $Port --baud $Baud --samples $Samples --spkp $SpeedKp --spki $SpeedKi --spiq $SpeedIq --olkp $CurrentKp --olki $CurrentKi
if ($LASTEXITCODE -ne 0) {
    throw "Encoder capture failed with exit code $LASTEXITCODE"
}
Write-Host "Encoder capture finished"
