# Sync editor navigation from the Keil target without changing the build.
param([string]$Target = 'Foc')

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$projectPath = Join-Path $repoRoot 'App/Project/foc.uvprojx'
$projectDir = Split-Path $projectPath -Parent
[xml]$project = Get-Content -LiteralPath $projectPath -Raw
$targets = @($project.Project.Targets.Target | Where-Object { $_.TargetName -eq $Target })
if ($targets.Count -ne 1) { throw "Keil target not found or ambiguous: $Target" }
$keilTarget = $targets[0]

function Get-CanonicalPath([string]$Path) {
    $full = [IO.Path]::GetFullPath($Path)
    $current = [IO.Path]::GetPathRoot($full)
    $rest = $full.Substring($current.Length)
    foreach ($part in ($rest -split '\\' | Where-Object { $_ })) {
        $match = @(Get-ChildItem -LiteralPath $current -Force | Where-Object { $_.Name -eq $part })
        if ($match.Count -ne 1) {
            throw "Cannot resolve path component '$part' under $current"
        }
        $current = $match[0].FullName
    }
    return $current
}

function ConvertTo-WorkspacePath([string]$Path) {
    $absolute = Get-CanonicalPath ([IO.Path]::GetFullPath((Join-Path $projectDir $Path)))
    $prefix = (Get-CanonicalPath $repoRoot).TrimEnd('\') + '\'
    if (!$absolute.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Path outside workspace: $Path"
    }
    return '${workspaceFolder}/' + $absolute.Substring($prefix.Length).Replace('\', '/')
}

$controls = $keilTarget.TargetOption.TargetArmAds.Cads.VariousControls
$includes = @($controls.IncludePath.Split(';') | Where-Object { $_ } | ForEach-Object {
    if (Test-Path -LiteralPath (Join-Path $projectDir $_) -PathType Container) {
        ConvertTo-WorkspacePath $_
    } else {
        Write-Warning "Skipping missing Keil include directory: $_"
    }
})
# Keil supplies CMSIS through its Pack; the editor uses the local copy.
$includes = @($includes + '${workspaceFolder}/Library/CMSIS/Include' | Select-Object -Unique)
$sourceDirs = @($keilTarget.Groups.Group | ForEach-Object {
    if ($_.GroupOption.CommonProperty.IncludeInBuild -eq '0') { return }
    $_.Files.File | ForEach-Object {
        if ($_.FileOption.CommonProperty.IncludeInBuild -eq '0') { return }
        if ($_.FilePath -and (Test-Path -LiteralPath (Join-Path $projectDir $_.FilePath) -PathType Leaf)) {
            ConvertTo-WorkspacePath (Split-Path $_.FilePath -Parent)
        }
    }
})
$browsePaths = @($sourceDirs + $includes | Sort-Object -Unique | ForEach-Object { "$_/*" })
$config = [ordered]@{
    configurations = @([ordered]@{
        name = "STM32F405 - $Target"
        compilerPath = ''
        intelliSenseMode = 'windows-clang-arm'
        cStandard = 'c99'
        cppStandard = 'c++11'
        includePath = $includes
        defines = @($controls.Define.Split(',') | ForEach-Object { $_.Trim() } | Where-Object { $_ })
        browse = [ordered]@{
            path = $browsePaths
            limitSymbolsToIncludedHeaders = $true
        }
    })
    version = 4
}
$configDir = Join-Path $repoRoot '.vscode'
New-Item -ItemType Directory -Force -Path $configDir | Out-Null
$outputPath = Join-Path $configDir 'c_cpp_properties.json'
$json = $config | ConvertTo-Json -Depth 8
[IO.File]::WriteAllText($outputPath, $json + [Environment]::NewLine, [Text.UTF8Encoding]::new($false))
Write-Host "Updated $outputPath ($($includes.Count) include directories, $($browsePaths.Count) index directories)."
