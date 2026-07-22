param(
    [switch]$KeepFrames
)

. (Join-Path $PSScriptRoot 'common.ps1')

$rawFrames = Join-Path $RepoRoot 'recordings\raw\warehouse_trial'
$video = Join-Path $RepoRoot 'recordings\warehouse_trial.mp4'
$thumbnail = Join-Path $RepoRoot 'recordings\warehouse_trial_thumbnail.png'
$recorder = Join-Path $RepoRoot 'tools\record_warehouse_demo.py'
$composer = Join-Path $RepoRoot 'tools\compose_warehouse_video.py'
$isaacPython = 'C:\isaacsim\python.bat'

if (-not (Test-Path $isaacPython)) {
    throw "Isaac Sim Python launcher not found: $isaacPython"
}

& $isaacPython $recorder --output-dir $rawFrames --fps 15
if ($LASTEXITCODE -ne 0) {
    throw "Isaac trial recording failed with exit code $LASTEXITCODE"
}

Push-Location $IsaacRosWorkspace
try {
    & $Pixi run python $composer --frames $rawFrames --output $video --thumbnail $thumbnail --fps 15
    if ($LASTEXITCODE -ne 0) {
        throw "Video composition failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

Write-Host "Trial video: $video" -ForegroundColor Green
Write-Host "Thumbnail: $thumbnail" -ForegroundColor Green

if (-not $KeepFrames) {
    $rawRoot = (Resolve-Path (Join-Path $RepoRoot 'recordings\raw')).Path
    $resolvedFrames = (Resolve-Path $rawFrames).Path
    $expectedPrefix = $rawRoot + [IO.Path]::DirectorySeparatorChar
    if (-not $resolvedFrames.StartsWith($expectedPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove capture outside the raw recordings directory: $resolvedFrames"
    }
    Remove-Item -LiteralPath $resolvedFrames -Recurse -Force
    Write-Host "Removed temporary raw frames: $resolvedFrames" -ForegroundColor DarkGray
}
