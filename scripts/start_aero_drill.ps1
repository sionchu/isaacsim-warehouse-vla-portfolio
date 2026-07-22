. (Join-Path $PSScriptRoot 'common.ps1')

$extensionRoot = Join-Path $RepoRoot 'isaacsim_exts'
$isaacSim = 'C:\isaacsim\isaac-sim.bat'
if (-not (Test-Path $isaacSim)) {
    throw "Isaac Sim standalone launcher not found: $isaacSim"
}

$command = "`"$isaacSim`" --ext-folder `"$extensionRoot`" --enable aero.drill.vla"
Push-Location $RepoRoot
try {
    & cmd.exe /d /s /c $command
} finally {
    Pop-Location
}
