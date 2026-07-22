from __future__ import annotations

import json

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

GOALS = {
    "B1": (4.0, 2.6),
    "B2": (4.0, 2.6),
    "B3": (4.0, 2.6),
    "B4": (4.0, 2.6),
    "B5": (4.0, 2.6),
    "C1": (4.0, -2.6),
    "C2": (4.0, -2.6),
    "C3": (4.0, -2.6),
    "C4": (4.0, -2.6),
    "C5": (4.0, -2.6),
}


class WarehouseDispatcher(Node):
    def __init__(self) -> None:
        super().__init__("warehouse_mission_dispatcher")
        self._action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._status = self.create_publisher(String, "/warehouse/mission_status", 10)
        self._request = self.create_subscription(
            String, "/warehouse/mission_request", self._on_request, 10
        )
        self._active_slot: str | None = None
        self.get_logger().info("Warehouse Nav2 dispatcher ready")

    def _on_request(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            slot = str(payload["slot"])
        except (json.JSONDecodeError, KeyError, TypeError) as error:
            self._publish("REJECTED", "", f"invalid request: {error}")
            return
        if slot not in GOALS:
            self._publish("REJECTED", slot, "unknown slot")
            return
        if self._active_slot is not None:
            self._publish("REJECTED", slot, f"mission {self._active_slot} is active")
            return
        if not self._action.wait_for_server(timeout_sec=2.0):
            self._publish("FAILED", slot, "Nav2 action server unavailable")
            return

        goal_x, goal_y = GOALS[slot]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.w = 1.0
        self._active_slot = slot
        self._publish("SENDING", slot, f"goal=({goal_x:.1f}, {goal_y:.1f})")
        future = self._action.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        slot = self._active_slot or ""
        try:
            goal_handle = future.result()
        except Exception as error:
            self._publish("FAILED", slot, str(error))
            self._active_slot = None
            return
        if not goal_handle.accepted:
            self._publish("REJECTED", slot, "Nav2 rejected the goal")
            self._active_slot = None
            return
        self._publish("ACTIVE", slot, "Nav2 goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result)

    def _result(self, future) -> None:
        slot = self._active_slot or ""
        try:
            status = future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self._publish("SUCCEEDED", slot, "robot reached the rack")
            elif status == GoalStatus.STATUS_CANCELED:
                self._publish("CANCELED", slot, "goal canceled")
            else:
                self._publish("FAILED", slot, f"Nav2 status={status}")
        except Exception as error:
            self._publish("FAILED", slot, str(error))
        finally:
            self._active_slot = None

    def _publish(self, status: str, slot: str, detail: str) -> None:
        message = String()
        message.data = json.dumps(
            {"status": status, "slot": slot, "detail": detail}, ensure_ascii=False
        )
        self._status.publish(message)
        self.get_logger().info(message.data)


def main() -> None:
    rclpy.init()
    node = WarehouseDispatcher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
