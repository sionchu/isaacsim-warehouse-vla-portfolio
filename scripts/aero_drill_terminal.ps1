param(
    [ValidateSet('hole', 'batch', 'monitor')]
    [string]$Action = 'hole',
    [ValidatePattern('^H(0[1-9]|10)$')]
    [string]$Hole = 'H01',
    [ValidateRange(10, 600)]
    [int]$TimeoutSeconds = 180
)

. (Join-Path $PSScriptRoot 'common.ps1')

if (-not (Test-Path -LiteralPath $PortfolioSetup)) {
    throw "ROS 2 package is not built. Run .\scripts\build.ps1 first."
}

$env:RMW_IMPLEMENTATION = 'rmw_fastrtps_cpp'
$env:ROS_DOMAIN_ID = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { '0' }
$terminalRunner = Join-Path $PSScriptRoot 'run_aero_drill_terminal.bat'
$command = "`"$terminalRunner`" --action $Action --hole $Hole --timeout $TimeoutSeconds"
Push-Location $IsaacRosWorkspace
try {
    & $Pixi run cmd.exe /d /s /c $command
    if ($LASTEXITCODE -ne 0) {
        throw "Aero drill ROS terminal exited with code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}
