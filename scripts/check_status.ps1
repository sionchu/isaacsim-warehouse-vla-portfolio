. (Join-Path $PSScriptRoot 'common.ps1')

if (-not (Test-Path $PortfolioSetup)) {
    throw 'Portfolio package is not built. Run .\scripts\build.ps1 first.'
}

$command = "call `"$PortfolioSetup`" && ros2 topic echo /portfolio/status std_msgs/msg/String --once --no-daemon --timeout 15"

Push-Location $IsaacRosWorkspace
try {
    & $Pixi run cmd.exe /d /s /c $command
} finally {
    Pop-Location
}
