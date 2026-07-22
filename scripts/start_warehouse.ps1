. (Join-Path $PSScriptRoot 'common.ps1')

$extensionRoot = Join-Path $RepoRoot 'isaacsim_exts'
$isaacSim = 'C:\isaacsim\isaac-sim.bat'
if (-not (Test-Path $isaacSim)) {
    throw "Isaac Sim standalone launcher not found: $isaacSim"
}

# Fast DDS is available in both the Pixi ROS 2 Jazzy environment and Isaac Sim.
$command = "set ROS_DISTRO=jazzy&& set RMW_IMPLEMENTATION=rmw_fastrtps_cpp&& `"$isaacSim`" --ext-folder `"$extensionRoot`" --enable warehouse.mission"

Push-Location $IsaacRosWorkspace
try {
    & $Pixi run cmd.exe /d /s /c $command
} finally {
    Pop-Location
}

