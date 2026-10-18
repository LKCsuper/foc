$ErrorActionPreference = 'Stop'
$root = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$source = Get-Content (Join-Path $root 'Scripts/build_keil.ps1') -Raw
$match = [regex]::Match($source, '(?s)    if \(\$Project -eq "Foc" -and \$Action -ne "clean"\) \{.*?\r?\n    \}')
if (!$match.Success) { throw 'ISR stack budget check missing' }
$check = [scriptblock]::Create($match.Value)
$folder = Join-Path $root '.build/host/stack_budget_fixture'
New-Item -ItemType Directory -Force $folder | Out-Null
$info = @{ OutHint = Join-Path $folder 'foc.axf' }
$Project = 'Foc'
$Action = 'build'
foreach ($case in @(
    @{ Stack = 1024; Depth = 1120; Missing = $false; Pass = $false },
    @{ Stack = 1631; Depth = 1120; Missing = $false; Pass = $false },
    @{ Stack = 1632; Depth = 1120; Missing = $false; Pass = $true },
    @{ Stack = 2048; Depth = 1120; Missing = $false; Pass = $true },
    @{ Stack = 2048; Depth = 1120; Missing = $true; Pass = $false }
)) {
    Set-Content (Join-Path $folder 'foc.map') "    STACK 0x2001f530 Section $($case.Stack) startup_stm32f40xx.o"
    $graph = if ($case.Missing) { '<P>No handler report</P>' } else {
        "<P><STRONG>DMA2_Stream4_IRQHandler</STRONG><BR>Max Depth = $($case.Depth)<P>Next"
    }
    Set-Content (Join-Path $folder 'foc.htm') $graph
    $passed = $true
    try { & $check } catch { $passed = $false }
    if ($passed -ne $case.Pass) { throw "Unexpected stack check result: $($case | ConvertTo-Json -Compress)" }
}
Write-Host 'test_isr_stack_budget: pass'
