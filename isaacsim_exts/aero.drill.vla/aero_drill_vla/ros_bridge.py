from __future__ import annotations

import json
import math
from collections import deque
from typing import Callable

import numpy as np


MISSION_TOPIC = "/aero_drill/mission_request"
ACK_TOPIC = "/aero_drill/command_ack"
STATUS_TOPIC = "/aero_drill/status"
JOINT_TOPIC = "/aero_drill/joint_states"
TCP_TOPIC = "/aero_drill/tcp_pose"


def _quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized ROS xyzw quaternion."""
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    quaternion /= max(float(np.linalg.norm(quaternion)), 1e-12)
    return tuple(float(value) for value in quaternion)


class AeroDrillRosBridge:
    """Bidirectional ROS 2 bridge for mission commands and live robot telemetry."""

    def __init__(self, command_callback: Callable[[dict], dict]) -> None:
        self.available = False
        self.error = ""
        self.events: deque[str] = deque(maxlen=10)
        self.publish_count = 0
        self.command_count = 0
        self._accumulator = 0.0
        self._sequence = 0
        self._last_state_key = ""
        self._owns_context = False
        self._node = None
        self._command_callback = command_callback
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String

            self._rclpy = rclpy
            self._string_type = String
            self._joint_type = JointState
            self._pose_type = PoseStamped
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_context = True
            self._node = rclpy.create_node("aero_drill_isaac_bridge")
            self._ack_publisher = self._node.create_publisher(String, ACK_TOPIC, 10)
            self._status_publisher = self._node.create_publisher(String, STATUS_TOPIC, 10)
            self._joint_publisher = self._node.create_publisher(JointState, JOINT_TOPIC, 10)
            self._tcp_publisher = self._node.create_publisher(PoseStamped, TCP_TOPIC, 10)
            self._command_subscriber = self._node.create_subscription(
                String,
                MISSION_TOPIC,
                self._on_command,
                10,
            )
            self.available = True
            self._event(f"[ROS] node ready: aero_drill_isaac_bridge")
            self._event(f"[SUB] {MISSION_TOPIC}  std_msgs/String")
        except Exception as error:
            self.error = str(error)
            self._event(f"[ROS] unavailable: {self.error}")

    def _event(self, text: str) -> None:
        self.events.append(text)
        print(text, flush=True)

    def _publish_string(self, publisher, payload: dict) -> None:
        message = self._string_type()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(message)

    def _on_command(self, message) -> None:
        self.command_count += 1
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise TypeError("command payload must be a JSON object")
            action = str(payload.get("action", "")).upper()
            hole = str(payload.get("hole", "--")).upper()
            self._event(f"[RX ] {MISSION_TOPIC} action={action} hole={hole}")
            result = dict(self._command_callback(payload))
            result.setdefault("accepted", True)
            result.setdefault("action", action)
            result.setdefault("hole", hole)
        except Exception as error:
            result = {
                "accepted": False,
                "action": "",
                "hole": "",
                "message": str(error),
            }
        result["command_count"] = self.command_count
        self._publish_string(self._ack_publisher, result)
        outcome = "ACCEPTED" if result.get("accepted") else "REJECTED"
        self._event(f"[TX ] {ACK_TOPIC} {outcome} {result.get('message', '')}")

    def spin_once(self) -> None:
        if self.available:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def tick(self, dt: float, controller, status_text: str) -> None:
        if not self.available:
            return
        self.spin_once()
        self._accumulator += max(float(dt), 0.0)
        if self._accumulator < 0.1:
            return
        self._accumulator %= 0.1
        self._sequence += 1
        self._publish_status(controller, status_text)
        self._publish_joint_state(controller)
        self._publish_tcp_pose(controller)
        self.publish_count += 3

    def _publish_status(self, controller, status_text: str) -> None:
        payload = {
            "sequence": self._sequence,
            "state": controller.state,
            "active_hole": controller.active_hole,
            "strategy": controller.active_strategy,
            "completed_count": controller.completed_count,
            "robot_ready": controller.robot_ready,
            "tcp_error_mm": round(controller.tcp_error_mm, 3),
            "clearance_mm": round(controller.clearance_mm, 3),
            "force_n": round(controller.current_force_n, 2),
            "spindle_rpm": controller.spindle_rpm,
            "quality": round(controller.last_quality, 2),
            "status": status_text,
        }
        self._publish_string(self._status_publisher, payload)
        state_key = (
            f"{payload['state']}:{payload['active_hole']}:{payload['completed_count']}"
        )
        if state_key != self._last_state_key:
            self._last_state_key = state_key
            self._event(
                f"[TX ] {STATUS_TOPIC} state={payload['state']} "
                f"hole={payload['active_hole']} done={payload['completed_count']}/10"
            )

    def _publish_joint_state(self, controller) -> None:
        message = self._joint_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "ur10e_base"
        rows = controller.joint_rows()
        message.name = [str(row["name"]) for row in rows]
        message.position = [math.radians(float(row["position_deg"])) for row in rows]
        message.velocity = [math.radians(float(row["velocity_deg_s"])) for row in rows]
        self._joint_publisher.publish(message)

    def _publish_tcp_pose(self, controller) -> None:
        message = self._pose_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = "world"
        if controller.robot is not None:
            position = controller.robot.current_tcp
            quaternion = _quaternion_xyzw(controller.robot.current_rotation)
            message.pose.position.x = float(position[0])
            message.pose.position.y = float(position[1])
            message.pose.position.z = float(position[2])
            message.pose.orientation.x = quaternion[0]
            message.pose.orientation.y = quaternion[1]
            message.pose.orientation.z = quaternion[2]
            message.pose.orientation.w = quaternion[3]
        else:
            message.pose.orientation.w = 1.0
        self._tcp_publisher.publish(message)

    def close(self) -> None:
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context and self._rclpy.ok():
            self._rclpy.shutdown()
