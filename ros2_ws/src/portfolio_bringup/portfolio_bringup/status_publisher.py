import json
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PortfolioStatusPublisher(Node):
    """Publish a lightweight heartbeat that proves the ROS environment is alive."""

    def __init__(self) -> None:
        super().__init__("portfolio_status")
        self.declare_parameter("project_name", "isaacsim_ros2_portfolio")
        self.declare_parameter("robot_name", "development_robot")
        self.declare_parameter("publish_rate_hz", 1.0)

        rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        self._publisher = self.create_publisher(String, "/portfolio/status", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_status)
        self.get_logger().info(f"Publishing /portfolio/status at {rate_hz:.1f} Hz")

    def _publish_status(self) -> None:
        payload = {
            "project": self.get_parameter("project_name").value,
            "robot": self.get_parameter("robot_name").value,
            "state": "ready",
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PortfolioStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

