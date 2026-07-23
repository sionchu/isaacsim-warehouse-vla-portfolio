param(
    [switch]$KeepFrames,
    [switch]$CleanupOnly,
    [ValidateRange(1, 10)]
    [int]$MaxHoles = 3
)

. (Join-Path $PSScriptRoot 'common.ps1')

$rawFrames = Join-Path $RepoRoot 'recordings\raw\aero_drill_trial'
$video = Join-Path $RepoRoot 'recordings\aero_drill_trial.mp4'
$thumbnail = Join-Path $RepoRoot 'recordings\aero_drill_trial_thumbnail.png'
$recorder = Join-Path $RepoRoot 'tools\record_aero_drill_demo.py'
$composer = Join-Path $RepoRoot 'tools\compose_aero_drill_video.py'
$isaacPython = 'C:\isaacsim\python.bat'

if (-not (Test-Path $isaacPython)) {
    throw "Isaac Sim Python launcher not found: $isaacPython"
}

if (-not $CleanupOnly) {
    & $isaacPython $recorder --output-dir $rawFrames --fps 15 --max-holes $MaxHoles
    if ($LASTEXITCODE -ne 0) {
        throw "Aero drill recording failed with exit code $LASTEXITCODE"
    }

    Push-Location $IsaacRosWorkspace
    try {
        & $Pixi run python $composer --frames $rawFrames --output $video --thumbnail $thumbnail --fps 15
        if ($LASTEXITCODE -ne 0) {
            throw "Aero drill video composition failed with exit code $LASTEXITCODE"
        }
    } finally {
        Pop-Location
    }

    Write-Host "Aero drill video: $video" -ForegroundColor Green
    Write-Host "Thumbnail: $thumbnail" -ForegroundColor Green
}

if (-not $KeepFrames -and (Test-Path -LiteralPath $rawFrames)) {
    $rawRoot = (Resolve-Path (Join-Path $RepoRoot 'recordings\raw')).Path
    $resolvedFrames = (Resolve-Path $rawFrames).Path
    $expectedPrefix = $rawRoot + [IO.Path]::DirectorySeparatorChar
    if (-not $resolvedFrames.StartsWith($expectedPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove capture outside the raw recordings directory: $resolvedFrames"
    }
    Remove-Item -LiteralPath $resolvedFrames -Recurse -Force
    Write-Host "Removed temporary raw frames: $resolvedFrames" -ForegroundColor DarkGray
}
