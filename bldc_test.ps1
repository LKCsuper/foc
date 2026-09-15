# Build, flash, capture, and analyze one bounded BLDC test.
param(
    [string]$Port = "COM18",
    [int]$Baud = 460800,
    [ValidateSet("build", "rebuild")]
    [string]$BuildAction = "build",
    [string]$Uv4 = "",
    [string]$JLink = "",
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

$script = Join-Path $PSScriptRoot "Scripts\bldc_build_flash_capture.ps1"
& $script -Port $Port -Baud $Baud -BuildAction $BuildAction -Uv4 $Uv4 `
    -JLink $JLink -Mode $Mode -Direction $Direction -Duty $Duty `
    -SampleSide $SampleSide -Duration $Duration -PeriodMs $PeriodMs -IntegralMvs $IntegralMvs
exit $LASTEXITCODE
