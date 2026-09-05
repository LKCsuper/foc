# 仓库根目录入口：转发到 Scripts/build_keil.ps1
# 用法: .\build_keil.ps1
#       .\build_keil.ps1 -Action rebuild
#       .\build_keil.ps1 -Project Boot
param(
    [ValidateSet("Foc", "Boot")]
    [string]$Project = "Foc",

    [ValidateSet("build", "rebuild", "clean")]
    [string]$Action = "build",

    [string]$Uv4 = "",

    [string]$Target = "",

    [switch]$StrictWarnings
)

$script = Join-Path $PSScriptRoot "Scripts\build_keil.ps1"
if (!(Test-Path -LiteralPath $script)) {
    throw "Missing $script"
}

& $script -Project $Project -Action $Action -Uv4 $Uv4 -Target $Target -StrictWarnings:$StrictWarnings
exit $LASTEXITCODE
