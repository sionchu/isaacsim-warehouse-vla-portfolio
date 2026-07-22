. (Join-Path $PSScriptRoot 'common.ps1')

Push-Location $IsaacRosWorkspace
try {
    & $Pixi run sim
} finally {
    Pop-Location
}

