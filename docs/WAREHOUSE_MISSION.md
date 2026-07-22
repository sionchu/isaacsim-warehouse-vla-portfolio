# Warehouse mission implementation notes

## Scenario

- A: loading and return zone at `(-6.0, 0.0)`
- B: five-level rack, approach pose `(4.0, 2.6)`
- C: five-level rack, approach pose `(4.0, -2.6)`
- Slots: `B1-B5` and `C1-C5`

Manual mode treats the selected slot as a hard safety constraint. Automatic mode masks occupied outputs and chooses only a free slot. If every slot is full, dispatch is rejected.

## Policy inputs and output

The local PyTorch policy receives:

1. A character-tokenized Korean or English instruction.
2. A `1 x 2 x 5` occupancy image, with one row per rack.

Its action head returns logits for ten discrete targets. The Isaac UI records instruction, selected target, policy source, confidence, elapsed time, and final occupancy as JSONL under `recordings/`.

## ROS 2 interface

| Topic | Type | Purpose |
|---|---|---|
| `/warehouse/mission_request` | `std_msgs/String` JSON | UI-to-Nav2 slot request |
| `/warehouse/mission_status` | `std_msgs/String` JSON | Accepted, active, success, or failure status |
| `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | Nova Carter 3D lidar |
| `/scan` | `sensor_msgs/LaserScan` | 2D projection for SLAM/Nav2 |
| `/chassis/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/map` | `nav_msgs/OccupancyGrid` | Online SLAM map |
| `/front_stereo_camera/left/image_raw` | `sensor_msgs/Image` | Operator vision panel |

`warehouse_dispatcher` converts B/C targets to `NavigateToPose` actions in the `map` frame. Shelf height is handled by the storage controller after arrival.

## Tested baseline

- Python compile check: passed
- ROS package build: passed
- Both launch descriptions: parsed successfully
- RViz dashboard YAML: parsed with six displays
- VLA-lite training: CUDA, 5,000 samples, 35 epochs, synthetic validation accuracy 0.999
- Isaac extension headless startup: passed, no Kit `[Error]` entries
- Isaac mission smoke: manual B3 and automatic B1 completed

## Next portfolio milestones

1. Replace the staged lift with a prismatic mast and contact-aware fork controller.
2. Collect MobilityGen routes with dynamic obstacles and compare learned waypoint proposals against Nav2.
3. Add camera-based slot occupancy instead of the internal occupancy state.
4. Save and reload SLAM maps, then add repeatable regression routes and success metrics.
5. Record a short demo video plus a rosbag/MCAP sensor trace for the portfolio release.
