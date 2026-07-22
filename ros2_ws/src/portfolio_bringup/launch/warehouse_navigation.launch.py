from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("portfolio_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    carter_share = get_package_share_directory("carter_navigation")
    slam_config = os.path.join(package_share, "config", "slam_toolbox.yaml")
    nav2_params = os.path.join(carter_share, "params", "carter_navigation_params.yaml")
    rviz_config = os.path.join(package_share, "rviz", "warehouse_dashboard.rviz")

    return LaunchDescription(
        [
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="warehouse_pointcloud_to_scan",
                remappings=[
                    ("cloud_in", "/front_3d_lidar/lidar_points"),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "target_frame": "front_3d_lidar",
                        "transform_tolerance": 0.01,
                        "min_height": -0.4,
                        "max_height": 1.5,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.0087,
                        "scan_time": 0.1,
                        "range_min": 0.1,
                        "range_max": 30.0,
                        "use_inf": True,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[slam_config],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "True",
                    "params_file": nav2_params,
                    "autostart": "True",
                }.items(),
            ),
            Node(
                package="portfolio_bringup",
                executable="warehouse_dispatcher",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
