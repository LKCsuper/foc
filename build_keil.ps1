# 仓库根目录入口：转发到 Scripts/build_keil.ps1
# 用法: .\build_keil.ps1
#       .\build_keil.ps1 -Action rebuild
#       .\build_keil.ps1 -Project Boot
#       .\build_keil.ps1 -Variant min -Defines "MOTOR_CFG_DIAG=0","MOTOR_CFG_CMD=0"
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

$script = Join-Path $PSScriptRoot "Scripts\build_keil.ps1"
if (!(Test-Path -LiteralPath $script)) {
    throw "Missing $script"
}

& $script -Project $Project -Action $Action -Uv4 $Uv4 -Target $Target -StrictWarnings:$StrictWarnings -Variant $Variant -Defines $Defines
exit $LASTEXITCODE
