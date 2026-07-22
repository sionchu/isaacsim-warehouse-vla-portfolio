from __future__ import annotations

import json
from typing import Callable


class RosMissionBridge:
    """Optional rclpy bridge between the Isaac UI and the Nav2 dispatcher."""

    def __init__(self, status_callback: Callable[[dict], None]) -> None:
        self.available = False
        self.error = ""
        self._owns_context = False
        self._node = None
        try:
            import rclpy
            from std_msgs.msg import String

            self._rclpy = rclpy
            self._message_type = String
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_context = True
            self._node = rclpy.create_node("warehouse_mission_isaac_ui")
            self._publisher = self._node.create_publisher(String, "/warehouse/mission_request", 10)
            self._subscriber = self._node.create_subscription(
                String,
                "/warehouse/mission_status",
                lambda message: status_callback(json.loads(message.data)),
                10,
            )
            self.available = True
        except Exception as error:
            self.error = str(error)

    def publish(self, slot_id: str, instruction: str, confidence: float, source: str) -> None:
        if not self.available:
            raise RuntimeError(f"ROS 2 mission bridge unavailable: {self.error}")
        message = self._message_type()
        message.data = json.dumps(
            {
                "slot": slot_id,
                "instruction": instruction,
                "confidence": confidence,
                "policy_source": source,
            },
            ensure_ascii=False,
        )
        self._publisher.publish(message)

    def spin_once(self) -> None:
        if self.available:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
