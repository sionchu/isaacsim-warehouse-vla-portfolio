. (Join-Path $PSScriptRoot 'common.ps1')

if (-not (Test-Path $PortfolioSetup)) {
    throw 'Portfolio package is not built. Run .\scripts\build.ps1 first.'
}

$command = "set RMW_IMPLEMENTATION=rmw_fastrtps_cpp&& call `"$PortfolioSetup`" && ros2 launch portfolio_bringup warehouse_monitor.launch.py"
Push-Location $IsaacRosWorkspace
try {
    & $Pixi run cmd.exe /d /s /c $command
} finally {
    Pop-Location
}
