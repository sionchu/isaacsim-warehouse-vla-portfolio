. (Join-Path $PSScriptRoot 'common.ps1')

Push-Location $IsaacRosWorkspace
try {
    & $Pixi run ros2 topic echo /clock rosgraph_msgs/msg/Clock --once --no-daemon --timeout 15
} finally {
    Pop-Location
}

