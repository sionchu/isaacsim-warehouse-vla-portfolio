. (Join-Path $PSScriptRoot 'common.ps1')

Push-Location $IsaacRosWorkspace
try {
    & $Pixi run zenoh
} finally {
    Pop-Location
}

