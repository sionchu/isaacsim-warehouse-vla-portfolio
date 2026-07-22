# Windows setup

## Installed layout

| Component | Location |
| --- | --- |
| Isaac Sim 6.0.1 | `C:\isaacsim` |
| Official Isaac Sim ROS workspaces | `C:\IsaacSim-ros_workspaces` |
| ROS 2 Jazzy Pixi environment | `C:\IsaacSim-ros_workspaces\jazzy_ws\.pixi` |
| Portfolio source | `C:\robotics-portfolio` |

The C: drive was selected because it is the faster Samsung NVMe and had substantially more free space than D: at installation time. Keep generated datasets and raw recordings outside Git and re-evaluate storage when either drive approaches 20% free space.

## Installed prerequisites

- NVIDIA display driver
- Git for Windows and Git LFS
- GitHub CLI
- Visual Studio 2022 Build Tools with Desktop C++ workload
- VS Code
- Pixi

Docker is not required for this Windows-native workflow. WSL2 Ubuntu 24.04 remains installed but is not used by the primary ROS 2 environment.

## Build

```powershell
cd C:\robotics-portfolio
.\scripts\build.ps1
```

## Start the stack

Start each command in a separate PowerShell terminal and keep it running:

```powershell
.\scripts\start_zenoh.ps1
.\scripts\start_isaac_sim.ps1
.\scripts\run_status.ps1
```

Within Isaac Sim, enable the ROS 2 bridge if necessary and create a Clock OmniGraph through `Tools > Robotics > ROS 2 OmniGraphs > Clock`. Press Play before checking `/clock`.

## Useful checks

```powershell
cd C:\IsaacSim-ros_workspaces\jazzy_ws
pixi run ros2 pkg list
pixi run ros2 topic list
pixi run rviz2
```

## GPU-conscious defaults

The RTX 4070 Ti has 12 GB VRAM, below the current 16 GB minimum listed for the newest Isaac Sim release. Use a single robot, modest sensor resolutions, and one or two active RTX sensors initially. Close browsers, games, and other GPU-heavy applications before launching the simulator.

