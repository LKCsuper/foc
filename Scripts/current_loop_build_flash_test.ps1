param(
    [string]$Port = "COM20",
    [int]$Baud = 2000000,
    [string]$JLink = "",
    [string]$Device = "STM32F405RG",
    [int]$SwdSpeedKhz = 100
)

$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
$firmware = Join-Path (Split-Path $root -Parent) "build\foc.hex"
$runDir = Join-Path $root (".build\current_loop_pipeline\" + (Get-Date -Format "yyyyMMdd-HHmmss"))
New-Item -ItemType Directory -Force -Path $runDir | Out-Null

function Resolve-JLinkPath {
    param([string]$Explicit)
    foreach ($candidate in @($Explicit, $env:JLINK_EXE,
            "D:\Users\15403\AppData\Local\Keil_v5\ARM\Segger\JLink.exe",
            "C:\Program Files\SEGGER\JLink\JLink.exe",
            "C:\Program Files (x86)\SEGGER\JLink\JLink.exe")) {
        if ($candidate -and (Test-Path -LiteralPath $candidate)) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    throw "JLink.exe not found"
}

Set-Location $root
Write-Host "[1/4] Stop motor"
python -X utf8 Scripts\foc_stop_serial.py --port $Port --baud $Baud

Write-Host "[2/4] Build firmware"
& (Join-Path $root "build_keil.ps1") -Project Foc -Action build
if ($LASTEXITCODE -ne 0 -or !(Test-Path -LiteralPath $firmware)) {
    throw "Keil build failed or firmware missing"
}

Write-Host "[3/4] Program and reset"
$jlinkPath = Resolve-JLinkPath $JLink
$jlinkScript = Join-Path $runDir "flash.jlink"
$jlinkLog = Join-Path $runDir "jlink.log"
@("connect", "r", "h", "loadfile $firmware", "r", "g", "q") |
    Set-Content -LiteralPath $jlinkScript -Encoding ASCII
& $jlinkPath -device $Device -if SWD -speed $SwdSpeedKhz -autoconnect 1 `
    -CommanderScript $jlinkScript 2>&1 | Tee-Object -FilePath $jlinkLog
$flashText = Get-Content -LiteralPath $jlinkLog -Raw
if ($LASTEXITCODE -ne 0 -or $flashText -notmatch "(?s)Downloading file.*?O\.K\." -or
    $flashText -match "(?im)^Error:") {
    throw "J-Link programming failed: $jlinkLog"
}

Write-Host "[4/4] Run bounded current-loop test"
Start-Sleep -Seconds 5
python -X utf8 Scripts\current_loop_test.py --port $Port --baud $Baud
if ($LASTEXITCODE -ne 0) {
    throw "Current-loop bench test failed"
}
