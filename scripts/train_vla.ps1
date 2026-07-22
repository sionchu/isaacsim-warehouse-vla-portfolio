. (Join-Path $PSScriptRoot 'common.ps1')

$trainer = Join-Path $RepoRoot 'ml\warehouse_vla\train_vla_lite.py'
Push-Location $IsaacRosWorkspace
try {
    & $Pixi run python $trainer --samples 5000 --epochs 35
    if ($LASTEXITCODE -ne 0) {
        throw "VLA-lite training failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

