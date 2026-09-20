# Build, flash, and run one bounded Hall FOC verification.
param(
    [string]$Port = "COM17",
    [int]$Baud = 460800,
    [ValidateSet("build", "rebuild")]
    [string]$BuildAction = "build",
    [string]$Uv4 = "",
    [string]$JLink = "",
    [int]$SwdSpeedKhz = 4000,
    [ValidateSet("up", "down", "start72", "brake", "reverse", "all")]
    [string]$Profile = "all",
    [ValidateSet("cw", "ccw")]
    [string]$Direction = "cw",
    [int]$MaxRpm = 800,
    [int]$Samples = 3
)

$script = Join-Path $PSScriptRoot "Scripts\hall_build_flash_verify.ps1"
& $script -Port $Port -Baud $Baud -BuildAction $BuildAction -Uv4 $Uv4 `
    -JLink $JLink -SwdSpeedKhz $SwdSpeedKhz -Profile $Profile -Direction $Direction `
    -MaxRpm $MaxRpm -Samples $Samples
exit $LASTEXITCODE
