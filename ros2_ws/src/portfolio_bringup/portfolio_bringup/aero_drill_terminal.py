from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

MISSION_TOPIC = "/aero_drill/mission_request"
ACK_TOPIC = "/aero_drill/command_ack"
STATUS_TOPIC = "/aero_drill/status"
JOINT_TOPIC = "/aero_drill/joint_states"
TCP_TOPIC = "/aero_drill/tcp_pose"


def parse_args(args=None):
    parser = argparse.ArgumentParser(description="Dispatch and monitor an Isaac Sim drilling mission.")
    parser.add_argument("--action", choices=("hole", "batch", "monitor"), default="hole")
    parser.add_argument("--hole", default="H01")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--event-log", type=Path)
    return parser.parse_known_args(args)


class AeroDrillTerminal(Node):
    def __init__(self, options) -> None:
        super().__init__("aero_drill_terminal")
        self.options = options
        self.started_at = time.monotonic()
        self.finished = False
        self.failed = False
        self.sent = False
        self.last_state_key = ""
        self.last_joint_print = 0.0
        self.last_pose_print = 0.0
        self.event_log = options.event_log.resolve() if options.event_log else None
        if self.event_log:
            self.event_log.parent.mkdir(parents=True, exist_ok=True)
            self.event_log.write_text("", encoding="utf-8")

        self.command_publisher = self.create_publisher(String, MISSION_TOPIC, 10)
        self.create_subscription(String, ACK_TOPIC, self._on_ack, 10)
        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(JointState, JOINT_TOPIC, self._on_joints, 10)
        self.create_subscription(PoseStamped, TCP_TOPIC, self._on_tcp, 10)
        self._print("[ROS2] aero_drill_terminal ready")
        self._print(f"[SUB ] {STATUS_TOPIC} | {JOINT_TOPIC} | {TCP_TOPIC} | {ACK_TOPIC}")
        if options.action == "monitor":
            self._print("[MODE] monitor only")
        else:
            target = options.hole.upper() if options.action == "hole" else "H01-H10"
            self._print(f"[WAIT] Isaac bridge + UR10e ready, target={target}")

    def _print(self, text: str) -> None:
        elapsed = time.monotonic() - self.started_at
        line = f"[{elapsed:06.2f}s] {text}"
        print(line, flush=True)
        if self.event_log:
            with self.event_log.open("a", encoding="utf-8") as stream:
                stream.write(
                    json.dumps(
                        {"elapsed": round(elapsed, 3), "line": text},
                        separators=(",", ":"),
                    )
                    + "\n"
                )

    def _dispatch(self) -> None:
        action = "RUN_HOLE" if self.options.action == "hole" else "RUN_BATCH"
        payload = {
            "action": action,
            "hole": self.options.hole.upper() if action == "RUN_HOLE" else "--",
            "instruction": (
                f"ROS 2 terminal command: process {self.options.hole.upper()}"
                if action == "RUN_HOLE"
                else "ROS 2 terminal command: process H01 through H10"
            ),
            "source": "portfolio_bringup/aero_drill_terminal",
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.command_publisher.publish(message)
        self.sent = True
        self._print(
            f"[PUB ] {MISSION_TOPIC} action={payload['action']} hole={payload['hole']}"
        )

    def _on_ack(self, message: String) -> None:
        payload = json.loads(message.data)
        result = "ACCEPTED" if payload.get("accepted") else "REJECTED"
        self._print(f"[ACK ] {result} {payload.get('message', '')}")
        if not payload.get("accepted"):
            self.failed = True
            self.finished = True

    def _on_status(self, message: String) -> None:
        payload = json.loads(message.data)
        state_key = (
            f"{payload.get('state')}:{payload.get('active_hole')}:"
            f"{payload.get('completed_count')}"
        )
        if state_key != self.last_state_key:
            self.last_state_key = state_key
            self._print(
                f"[STAT] state={payload.get('state')} hole={payload.get('active_hole')} "
                f"done={payload.get('completed_count')}/10 "
                f"tcp={float(payload.get('tcp_error_mm', 0.0)):.2f}mm"
            )
        if (
            self.options.action != "monitor"
            and not self.sent
            and payload.get("robot_ready")
            and payload.get("state") == "IDLE"
        ):
            self._dispatch()

        completed = int(payload.get("completed_count", 0))
        target = 1 if self.options.action == "hole" else 10
        if (
            self.sent
            and self.options.action != "monitor"
            and payload.get("state") == "IDLE"
            and completed >= target
        ):
            self._print(f"[DONE] mission complete, completed={completed}")
            self.finished = True

    def _on_joints(self, message: JointState) -> None:
        now = time.monotonic()
        if now - self.last_joint_print < 1.0:
            return
        self.last_joint_print = now
        values = " ".join(
            f"J{index + 1}={math.degrees(value):+.1f}deg"
            for index, value in enumerate(message.position[:6])
        )
        self._print(f"[JOINT] {values}")

    def _on_tcp(self, message: PoseStamped) -> None:
        now = time.monotonic()
        if now - self.last_pose_print < 1.0:
            return
        self.last_pose_print = now
        point = message.pose.position
        self._print(f"[TCP ] W=({point.x:+.3f},{point.y:+.3f},{point.z:+.3f})m")

    @property
    def timed_out(self) -> bool:
        return time.monotonic() - self.started_at >= self.options.timeout


def main(args=None) -> None:
    options, ros_args = parse_args(args)
    options.hole = options.hole.upper()
    if not options.hole.startswith("H") or not options.hole[1:].isdigit():
        raise SystemExit(f"Invalid hole id: {options.hole}")
    rclpy.init(args=ros_args)
    node = AeroDrillTerminal(options)
    try:
        while rclpy.ok() and not node.finished and not node.timed_out:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.timed_out:
            node._print("[FAIL] timeout waiting for mission completion")
            node.failed = True
    except KeyboardInterrupt:
        node._print("[STOP] interrupted")
    finally:
        failed = node.failed
        node.destroy_node()
        rclpy.shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
