from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="portfolio_bringup",
                executable="status_publisher",
                name="portfolio_status",
                output="screen",
            )
        ]
    )

