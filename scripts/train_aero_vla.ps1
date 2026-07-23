. (Join-Path $PSScriptRoot 'common.ps1')

$trainer = Join-Path $RepoRoot 'ml\aero_drill_vla\train_aero_vla_lite.py'
Push-Location $IsaacRosWorkspace
try {
    & $Pixi run python $trainer --samples 6000 --epochs 35
    if ($LASTEXITCODE -ne 0) {
        throw "Aero VLA-lite training failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}
