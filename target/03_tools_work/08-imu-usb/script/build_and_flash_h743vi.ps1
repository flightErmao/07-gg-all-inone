param(
    [bool]$DoBuild = $true,
    [bool]$DoFlash = $true
)

[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
$OutputEncoding = [Console]::OutputEncoding
$ErrorActionPreference = "Stop"

function Resolve-ToolPath {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CommandName,
        [string[]]$CandidatePaths = @()
    )

    $command = Get-Command $CommandName -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -ne $command -and $command.Source) {
        return $command.Source
    }

    foreach ($candidate in $CandidatePaths) {
        if ([string]::IsNullOrWhiteSpace($candidate)) {
            continue
        }
        if (Test-Path -LiteralPath $candidate) {
            return $candidate
        }
    }

    throw "Tool not found: $CommandName"
}

$projectDir = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$buildDir = Join-Path $projectDir "build"
$binPath = Join-Path $buildDir "rtthread.bin"
$jlinkScript = Join-Path $buildDir "jlink_flash_h743vi.jlink"

if (-not (Test-Path -LiteralPath $jlinkScript)) {
    throw "JLink command file not found: $jlinkScript"
}

$sconsPath = Resolve-ToolPath -CommandName "scons" -CandidatePaths @(
    (Join-Path $env:LOCALAPPDATA "Programs\Python\Python311\Scripts\scons.exe")
)

$jlinkPath = Resolve-ToolPath -CommandName "JLink.exe" -CandidatePaths @(
    "C:\Keil_v5\ARM\Segger\JLink.exe",
    "C:\0tools\54-rtt-study\RT-ThreadStudio\repo\Extract\Debugger_Support_Packages\SEGGER\J-Link\v8.24\JLink.exe",
    "C:\0tools\54-rtt-study\RT-ThreadStudio\repo\Extract\Debugger_Support_Packages\SEGGER\J-Link\v7.92\JLink.exe",
    "C:\0tools\54-rtt-study\RT-ThreadStudio\repo\Extract\Debugger_Support_Packages\SEGGER\J-Link\v7.50a\JLink.exe"
)

Write-Output "Project dir: $projectDir"
Write-Output "Build tool: $sconsPath"
Write-Output "Flash tool: $jlinkPath"

Push-Location $projectDir
try {
    if ($DoBuild) {
        Write-Output "Starting build..."
        & $sconsPath "-j8"
        if ($LASTEXITCODE -ne 0) {
            throw "Build failed with exit code: $LASTEXITCODE"
        }
    } else {
        Write-Output "Build skipped"
    }

    if (-not (Test-Path -LiteralPath $binPath)) {
        throw "Firmware binary not found: $binPath"
    }

    if ($DoFlash) {
        Write-Output "Starting flash for STM32H743VI..."
        & $jlinkPath "-CommandFile" $jlinkScript
        if ($LASTEXITCODE -ne 0) {
            throw "Flash failed with exit code: $LASTEXITCODE"
        }
    } else {
        Write-Output "Flash skipped"
    }

    Write-Output "Build and flash completed"
}
finally {
    Pop-Location
}
