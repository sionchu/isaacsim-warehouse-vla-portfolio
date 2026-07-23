param(
    [switch]$KeepFrames,
    [switch]$CleanupOnly,
    [ValidatePattern('^H(0[1-9]|10)$')]
    [string]$Hole = 'H01'
)

. (Join-Path $PSScriptRoot 'common.ps1')

$rawFrames = Join-Path $RepoRoot 'recordings\raw\aero_drill_ros_full'
$video = Join-Path $RepoRoot 'recordings\aero_drill_ros_full.mp4'
$thumbnail = Join-Path $RepoRoot 'recordings\aero_drill_ros_full_thumbnail.png'
$terminalLog = Join-Path $rawFrames 'ros_terminal.jsonl'
$recorder = Join-Path $RepoRoot 'tools\record_aero_drill_demo.py'
$composer = Join-Path $RepoRoot 'tools\compose_aero_drill_ros_video.py'
$terminalRunner = Join-Path $PSScriptRoot 'run_aero_drill_terminal.bat'
$isaacPython = 'C:\isaacsim\python.bat'

if (-not (Test-Path -LiteralPath $isaacPython)) {
    throw "Isaac Sim Python launcher not found: $isaacPython"
}

if ($CleanupOnly) {
    if (Test-Path -LiteralPath $rawFrames) {
        $rawRoot = (Resolve-Path (Join-Path $RepoRoot 'recordings\raw')).Path
        $resolvedFrames = (Resolve-Path $rawFrames).Path
        $expectedPrefix = $rawRoot + [IO.Path]::DirectorySeparatorChar
        if (-not $resolvedFrames.StartsWith($expectedPrefix, [StringComparison]::OrdinalIgnoreCase)) {
            throw "Refusing to remove capture outside the raw recordings directory: $resolvedFrames"
        }
        Remove-Item -LiteralPath $resolvedFrames -Recurse -Force
        Write-Host "Removed temporary ROS capture: $resolvedFrames" -ForegroundColor DarkGray
    }
    return
}

if (-not (Test-Path -LiteralPath $PortfolioSetup)) {
    & (Join-Path $PSScriptRoot 'build.ps1')
}

$env:ROS_DISTRO = 'jazzy'
$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { '0' }
$rosInternalLib = 'C:\isaacsim\exts\isaacsim.ros2.core\jazzy\lib'
if (-not (($env:Path -split ';') -contains $rosInternalLib)) {
    $env:Path = "$env:Path;$rosInternalLib"
}
New-Item -ItemType Directory -Force -Path $rawFrames | Out-Null

$terminalCommand = (
    "`"$terminalRunner`" " +
    "--action hole --hole $Hole --timeout 240 --event-log `"$terminalLog`""
)
$terminalJob = Start-Job -ScriptBlock {
    param($pixiPath, $workspace, $command, $domainId)
    $env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
    $env:ROS_DOMAIN_ID = $domainId
    Set-Location -LiteralPath $workspace
    & $pixiPath run cmd.exe /d /s /c $command
} -ArgumentList $Pixi, $IsaacRosWorkspace, $terminalCommand, $env:ROS_DOMAIN_ID

try {
    & $isaacPython $recorder --output-dir $rawFrames --fps 15 --max-holes 1 --ros
    if ($LASTEXITCODE -ne 0) {
        throw "ROS aerospace drill recording failed with exit code $LASTEXITCODE"
    }

    Wait-Job -Job $terminalJob -Timeout 30 | Out-Null
    $terminalOutput = Receive-Job -Job $terminalJob -ErrorAction SilentlyContinue
    if ($terminalOutput) {
        $terminalOutput | ForEach-Object { Write-Host $_ }
    }
    if ($terminalJob.State -ne 'Completed') {
        throw "ROS terminal did not observe mission completion."
    }

    Push-Location $IsaacRosWorkspace
    try {
        & $Pixi run python $composer `
            --frames $rawFrames `
            --output $video `
            --thumbnail $thumbnail `
            --fps 15
        if ($LASTEXITCODE -ne 0) {
            throw "ROS aerospace drill video composition failed with exit code $LASTEXITCODE"
        }
    } finally {
        Pop-Location
    }

    Write-Host "ROS closed-loop video: $video" -ForegroundColor Green
    Write-Host "Thumbnail: $thumbnail" -ForegroundColor Green
} finally {
    if ($terminalJob.State -notin @('Completed', 'Failed', 'Stopped')) {
        Stop-Job -Job $terminalJob
    }
    Remove-Job -Job $terminalJob -Force
}

if (-not $KeepFrames -and (Test-Path -LiteralPath $rawFrames)) {
    $rawRoot = (Resolve-Path (Join-Path $RepoRoot 'recordings\raw')).Path
    $resolvedFrames = (Resolve-Path $rawFrames).Path
    $expectedPrefix = $rawRoot + [IO.Path]::DirectorySeparatorChar
    if (-not $resolvedFrames.StartsWith($expectedPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove capture outside the raw recordings directory: $resolvedFrames"
    }
    Remove-Item -LiteralPath $resolvedFrames -Recurse -Force
    Write-Host "Removed temporary ROS frames: $resolvedFrames" -ForegroundColor DarkGray
}
