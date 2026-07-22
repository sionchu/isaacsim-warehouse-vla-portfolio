# Isaac Sim + ROS 2 Robotics Portfolio

A reproducible Windows 11 robotics portfolio workspace built around NVIDIA Isaac Sim 6.0.1, ROS 2 Jazzy, Zenoh, Nav2, MoveIt 2, and Python.

## Local toolchain

- GPU: NVIDIA GeForce RTX 4070 Ti (12 GB VRAM)
- Isaac Sim: 6.0.1
- ROS 2: Jazzy
- Middleware: `rmw_zenoh_cpp`
- Dependency manager: Pixi
- Official ROS workspace: `C:\IsaacSim-ros_workspaces\jazzy_ws`
- Portfolio repository: `C:\robotics-portfolio`

Large simulator binaries, generated build trees, recordings, and datasets are intentionally excluded from Git. Binary portfolio assets can be versioned with Git LFS.

## Quick start

Open PowerShell and build the portfolio package once:

```powershell
cd C:\robotics-portfolio
.\scripts\build.ps1
```

Then use three terminals:

```powershell
# Terminal 1: ROS 2 discovery router
.\scripts\start_zenoh.ps1

# Terminal 2: Isaac Sim with the ROS 2 bridge enabled
.\scripts\start_isaac_sim.ps1

# Terminal 3: portfolio status node
.\scripts\run_status.ps1

# Terminal 4: receive one status heartbeat
.\scripts\check_status.ps1
```

To verify the bridge, create a ROS 2 Clock OmniGraph in Isaac Sim, press Play, and run:

```powershell
.\scripts\check_clock.ps1
```

## Repository layout

```text
config/                         Project-level configuration
docs/                           Setup notes and portfolio roadmap
ros2_ws/src/portfolio_bringup/  ROS 2 package owned by this repository
scripts/                        Reproducible build and launch commands
```

See [Windows setup](docs/SETUP_WINDOWS.md), [portfolio roadmap](docs/PORTFOLIO_PLAN.md), and [GitHub publishing](docs/GITHUB.md).

## License

Source code in this repository is available under the Apache License 2.0. Third-party robot models, environments, and NVIDIA assets retain their respective licenses and should not be copied into this repository unless redistribution is permitted.

