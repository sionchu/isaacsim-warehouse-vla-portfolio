$ErrorActionPreference = 'Stop'

# Pick up tools installed by winget even when this script is launched from an
# older terminal that has not inherited the updated user PATH yet.
$machinePath = [Environment]::GetEnvironmentVariable('Path', 'Machine')
$userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
$env:Path = "$machinePath;$userPath"

$script:RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$script:IsaacRosWorkspace = if ($env:ISAAC_ROS_WS) {
    $env:ISAAC_ROS_WS
} else {
    'C:\IsaacSim-ros_workspaces\jazzy_ws'
}

if (-not (Test-Path $script:IsaacRosWorkspace)) {
    throw "Isaac Sim ROS workspace not found: $script:IsaacRosWorkspace"
}

$script:Pixi = (Get-Command pixi -ErrorAction Stop).Source
$script:PortfolioSetup = Join-Path $script:RepoRoot 'ros2_ws\install\setup.bat'
