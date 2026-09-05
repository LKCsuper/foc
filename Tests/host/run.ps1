param([string]$Compiler = "")
$ErrorActionPreference = "Stop"
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
Push-Location $root
try {
    New-Item -ItemType Directory -Force .build/host | Out-Null
    $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $root '.build/zig-cache'
    if (!$Compiler) {
        $Compiler = Join-Path $root '.build/tools/ziglang/zig.exe'
        if (!(Test-Path $Compiler)) { throw 'Specify -Compiler path to zig.exe, or install ziglang into .build/tools.' }
    }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -Wno-unused-variable -D_DEFAULT_SOURCE -IApp/motor_param/inc -IApp/motor/inc -ILibrary/Common/inc App/motor_param/motor_settings.c App/motor_param/motor_param.c Tests/host/test_motor_settings.c -o .build/host/test_motor_settings.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host compilation failed' }
    & .build/host/test_motor_settings.exe
    if ($LASTEXITCODE -ne 0) { throw 'Host tests failed' }
    & $Compiler cc -std=c99 -Wall -Wextra -Werror -Wno-int-to-void-pointer-cast -ITests/host/stubs -IApp/motor_param/inc -IApp/motor/inc App/motor_param/motor_settings.c App/motor_param/flash_storage.c Tests/host/test_flash_storage.c -o .build/host/test_flash_storage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flash host compilation failed' }
    & .build/host/test_flash_storage.exe
    if ($LASTEXITCODE -ne 0) { throw 'Flash host tests failed' }
} finally { Pop-Location }
