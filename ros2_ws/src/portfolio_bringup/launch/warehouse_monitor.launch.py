from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rviz_config = os.path.join(
        get_package_share_directory("portfolio_bringup"), "rviz", "warehouse_dashboard.rviz"
    )
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
