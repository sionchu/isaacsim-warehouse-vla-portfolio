. (Join-Path $PSScriptRoot 'common.ps1')

$extensionRoot = Join-Path $RepoRoot 'isaacsim_exts'
$isaacSim = 'C:\isaacsim\isaac-sim.bat'
if (-not (Test-Path -LiteralPath $isaacSim)) {
    throw "Isaac Sim standalone launcher not found: $isaacSim"
}

$env:ROS_DISTRO = $null
$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { '0' }
$rosInternalLib = 'C:\isaacsim\exts\isaacsim.ros2.core\jazzy\lib'
if (-not (($env:Path -split ';') -contains $rosInternalLib)) {
    $env:Path = "$env:Path;$rosInternalLib"
}
$command = (
    "`"$isaacSim`" --ext-folder `"$extensionRoot`" --enable aero.drill.vla " +
    "--/exts/isaacsim.ros2.bridge/ros_distro=jazzy"
)

Push-Location $RepoRoot
try {
    & cmd.exe /d /s /c $command
} finally {
    Pop-Location
}
