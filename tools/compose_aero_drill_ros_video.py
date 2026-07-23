from __future__ import annotations

import argparse
import json
from pathlib import Path

import av
import numpy as np
from PIL import Image, ImageDraw, ImageFont

WIDTH = 1600
HEIGHT = 900
VIEW_BOX = (18, 72, 1016, 650)
TERM_BOX = (1034, 72, 1582, 650)
COLORS = {
    "background": (9, 14, 20),
    "panel": (20, 29, 38),
    "terminal": (8, 12, 16),
    "line": (58, 76, 94),
    "white": (237, 244, 250),
    "muted": (145, 163, 180),
    "cyan": (39, 214, 226),
    "green": (51, 220, 132),
    "orange": (255, 164, 49),
    "red": (244, 80, 90),
}


def font(size: int, bold: bool = False, mono: bool = False):
    fonts = Path("C:/Windows/Fonts")
    if mono:
        path = fonts / ("consolab.ttf" if bold else "consola.ttf")
    else:
        path = fonts / ("segoeuib.ttf" if bold else "segoeui.ttf")
    return ImageFont.truetype(str(path), size) if path.exists() else ImageFont.load_default()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--thumbnail", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=15)
    return parser.parse_args()


def terminal_color(line: str):
    if "[RX " in line or "[ACK" in line:
        return COLORS["orange"]
    if "[TX " in line or "[PUB" in line:
        return COLORS["cyan"]
    if "DONE" in line or "ready" in line:
        return COLORS["green"]
    if "FAIL" in line or "REJECT" in line:
        return COLORS["red"]
    return COLORS["white"]


def draw_terminal(draw: ImageDraw.ImageDraw, data: dict) -> None:
    draw.rounded_rectangle(TERM_BOX, radius=8, fill=COLORS["terminal"], outline=COLORS["line"], width=2)
    draw.rectangle((1035, 73, 1581, 113), fill=(28, 37, 47))
    draw.circle((1055, 93), 6, fill=COLORS["red"])
    draw.circle((1076, 93), 6, fill=COLORS["orange"])
    draw.circle((1097, 93), 6, fill=COLORS["green"])
    draw.text((1120, 83), "PowerShell  |  ROS 2 Jazzy / Fast DDS", font=font(15, True), fill=COLORS["white"])
    lines = list(data.get("ros_terminal", []))[-10:]
    y = 130
    for line in lines:
        clipped = line if len(line) <= 65 else line[:62] + "..."
        draw.text((1052, y), clipped, font=font(15, mono=True), fill=terminal_color(line))
        y += 43
    prompt = "PS C:\\robotics-portfolio> ros2 topic echo /aero_drill/status"
    draw.text((1052, 618), prompt, font=font(13, mono=True), fill=COLORS["muted"])


def draw_holes(draw: ImageDraw.ImageDraw, data: dict) -> None:
    active = data["active_hole"]
    for index in range(10):
        hole = f"H{index + 1:02d}"
        x = 42 + index * 82
        fill = (42, 54, 66)
        if data["completed"].get(hole):
            fill = COLORS["green"]
        elif hole == active:
            fill = COLORS["orange"]
        draw.rounded_rectangle((x, 714, x + 68, 752), radius=6, fill=fill, outline=COLORS["line"])
        draw.text((x + 34, 733), hole, font=font(14, True), fill=(7, 13, 18), anchor="mm")


def compose(raw: Image.Image, data: dict, index: int, fps: int) -> Image.Image:
    canvas = Image.new("RGB", (WIDTH, HEIGHT), COLORS["background"])
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (20, 18),
        "ROS 2 CLOSED-LOOP AEROSPACE DRILLING | OFFICIAL UR10e",
        font=font(27, True),
        fill=COLORS["white"],
    )
    draw.text((1578, 28), f"T+{index / fps:05.1f}s", font=font(16), fill=COLORS["muted"], anchor="ra")

    viewport = raw.convert("RGB").resize(
        (VIEW_BOX[2] - VIEW_BOX[0], VIEW_BOX[3] - VIEW_BOX[1]),
        Image.Resampling.LANCZOS,
    )
    canvas.paste(viewport, (VIEW_BOX[0], VIEW_BOX[1]))
    draw.rounded_rectangle(VIEW_BOX, radius=8, outline=COLORS["line"], width=2)
    draw.text(
        (34, 88),
        "ISAAC SIM 6 | UR10e + cuMotion + COLLISION WORLD",
        font=font(14, True),
        fill=COLORS["white"],
    )
    draw_terminal(draw, data)

    draw.rounded_rectangle((18, 670, 1582, 880), radius=9, fill=COLORS["panel"], outline=COLORS["line"])
    draw.text(
        (34, 682),
        "ROS CLI",
        font=font(15, True),
        fill=COLORS["orange"],
    )
    draw.text(
        (112, 682),
        "→ /mission_request → ISAAC MISSION CONTROLLER → /status + /joint_states + /tcp_pose → ROS CLI",
        font=font(15, True),
        fill=COLORS["cyan"],
    )
    draw_holes(draw, data)

    state = data["state"]
    metric = (
        f"STATE {state}   HOLE {data['active_hole']}   "
        f"TCP ERROR {data.get('tcp_error_mm', 0.0):.2f} mm   "
        f"CLEARANCE {data.get('clearance_mm', 0.0):+.1f} mm   "
        f"FORCE {data['force_n']:.1f} N   RPM {data['spindle_rpm']}   "
        f"ROS RX {data.get('ros_command_count', 0)} / TX {data.get('ros_publish_count', 0)}"
    )
    draw.text((36, 772), metric, font=font(17, True), fill=COLORS["white"])
    joints = "  ".join(
        f"J{i + 1} {float(row['position_deg']):+.1f}°"
        for i, row in enumerate(data.get("joints", []))
    )
    draw.text((36, 810), joints, font=font(16, True, mono=True), fill=COLORS["cyan"])
    draw.text((36, 846), data["status"], font=font(15), fill=COLORS["muted"])
    return canvas


def main() -> None:
    args = parse_args()
    frame_paths = sorted(args.frames.glob("frame_*.png"))
    telemetry = json.loads((args.frames / "telemetry.json").read_text(encoding="utf-8"))
    if not frame_paths or len(frame_paths) != len(telemetry):
        raise RuntimeError(f"frame/telemetry mismatch: {len(frame_paths)} != {len(telemetry)}")
    if not any(item.get("ros_enabled") for item in telemetry):
        raise RuntimeError("capture does not contain ROS 2 bridge telemetry")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.thumbnail.parent.mkdir(parents=True, exist_ok=True)
    container = av.open(str(args.output), mode="w")
    stream = container.add_stream("libx264", rate=args.fps)
    stream.width = WIDTH
    stream.height = HEIGHT
    stream.pix_fmt = "yuv420p"
    stream.options = {"crf": "20", "preset": "medium"}
    thumbnail_index = min(max(len(frame_paths) // 2, 0), len(frame_paths) - 1)
    for index, (path, data) in enumerate(zip(frame_paths, telemetry)):
        frame = compose(Image.open(path), data, index, args.fps)
        if index == thumbnail_index:
            frame.save(args.thumbnail, quality=92)
        video_frame = av.VideoFrame.from_ndarray(np.asarray(frame), format="rgb24")
        for packet in stream.encode(video_frame):
            container.mux(packet)
    for packet in stream.encode():
        container.mux(packet)
    container.close()
    print(f"video={args.output}")
    print(f"thumbnail={args.thumbnail}")


if __name__ == "__main__":
    main()
