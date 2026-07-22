. (Join-Path $PSScriptRoot 'common.ps1')

$workspace = Join-Path $RepoRoot 'ros2_ws'
$source = Join-Path $workspace 'src'
$build = Join-Path $workspace 'build'
$install = Join-Path $workspace 'install'
$log = Join-Path $workspace 'log'

Push-Location $IsaacRosWorkspace
try {
    & $Pixi run colcon --log-base $log build `
        --merge-install `
        --base-paths $source `
        --build-base $build `
        --install-base $install `
        --cmake-args -DBUILD_TESTING=OFF
    if ($LASTEXITCODE -ne 0) {
        throw "Portfolio ROS 2 build failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

Write-Host "Portfolio ROS 2 package built successfully." -ForegroundColor Green

