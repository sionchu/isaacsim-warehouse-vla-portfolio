. (Join-Path $PSScriptRoot 'common.ps1')

if (-not (Test-Path $PortfolioSetup)) {
    throw 'Portfolio package is not built. Run .\scripts\build.ps1 first.'
}

$config = Join-Path $RepoRoot 'config\portfolio.yaml'
$command = "call `"$PortfolioSetup`" && ros2 run portfolio_bringup status_publisher --ros-args --params-file `"$config`""

Push-Location $IsaacRosWorkspace
try {
    & $Pixi run cmd.exe /d /s /c $command
} finally {
    Pop-Location
}

