from __future__ import annotations

import argparse
import json
from pathlib import Path

import av
import numpy as np
from PIL import Image, ImageDraw, ImageFont

WIDTH = 1280
HEIGHT = 720
VIEW_BOX = (18, 76, 900, 604)
PANEL_X = 918
COLORS = {
    "background": (12, 18, 25),
    "panel": (24, 33, 43),
    "line": (63, 79, 96),
    "white": (237, 244, 250),
    "muted": (150, 167, 184),
    "cyan": (36, 211, 224),
    "green": (45, 215, 126),
    "orange": (255, 164, 49),
    "red": (244, 73, 83),
}


def font(size: int, bold: bool = False):
    path = Path("C:/Windows/Fonts") / ("segoeuib.ttf" if bold else "segoeui.ttf")
    return ImageFont.truetype(str(path), size) if path.exists() else ImageFont.load_default()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--thumbnail", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=15)
    return parser.parse_args()


def draw_hole_map(draw: ImageDraw.ImageDraw, data: dict) -> None:
    draw.text((PANEL_X + 16, 98), "DRPE HOLE MAP", font=font(17, True), fill=COLORS["white"])
    active = data["active_hole"]
    for index in range(10):
        row, column = divmod(index, 5)
        hole = f"H{index + 1:02d}"
        x = PANEL_X + 17 + column * 66
        y = 132 + row * 52
        fill = (41, 53, 66)
        outline = COLORS["line"]
        if data["completed"].get(hole):
            fill = COLORS["green"]
        elif hole == active:
            fill = COLORS["orange"]
            outline = COLORS["white"]
        draw.rounded_rectangle((x, y, x + 54, y + 38), radius=7, fill=fill, outline=outline, width=2)
        draw.text((x + 27, y + 19), hole, font=font(14, True), fill=(9, 17, 23), anchor="mm")


def metric(draw: ImageDraw.ImageDraw, y: int, label: str, value: str, accent=None) -> None:
    draw.text((PANEL_X + 17, y), label, font=font(14), fill=COLORS["muted"])
    draw.text((1254, y), value, font=font(16, True), fill=accent or COLORS["white"], anchor="ra")


def compose(raw: Image.Image, data: dict, index: int, fps: int) -> Image.Image:
    canvas = Image.new("RGB", (WIDTH, HEIGHT), COLORS["background"])
    draw = ImageDraw.Draw(canvas)
    draw.text((20, 18), "UR10e AEROSPACE DRILLING VLA DIGITAL TWIN", font=font(28, True), fill=COLORS["white"])
    draw.text((1260, 28), f"T+{index / fps:05.1f}s", font=font(17), fill=COLORS["muted"], anchor="ra")

    viewport = raw.convert("RGB").resize((VIEW_BOX[2] - VIEW_BOX[0], VIEW_BOX[3] - VIEW_BOX[1]), Image.Resampling.LANCZOS)
    canvas.paste(viewport, (VIEW_BOX[0], VIEW_BOX[1]))
    draw.rounded_rectangle(VIEW_BOX, radius=8, outline=COLORS["line"], width=2)
    draw.text((32, 90), "ISAAC SIM | OFFICIAL UR10e + SCALED DRPE PANEL", font=font(14, True), fill=COLORS["white"])

    draw.rounded_rectangle((PANEL_X, 76, 1262, 604), radius=10, fill=COLORS["panel"], outline=COLORS["line"])
    draw_hole_map(draw, data)
    draw.line((PANEL_X + 16, 250, 1246, 250), fill=COLORS["line"], width=1)
    metric(draw, 270, "ACTIVE HOLE", data["active_hole"], COLORS["orange"])
    metric(draw, 304, "STATE", data["state"], COLORS["cyan"])
    metric(draw, 338, "STRATEGY", data["strategy"])
    metric(draw, 372, "TCP ERROR", f"{data.get('tcp_error_mm', 0.0):.1f} mm")
    metric(draw, 406, "CLEARANCE", f"{data.get('clearance_mm', 0.0):+.1f} mm")
    metric(draw, 440, "AXIAL FORCE", f"{data['force_n']:.1f} N")
    metric(draw, 474, "SPINDLE", f"{data['spindle_rpm']} rpm")
    metric(draw, 508, "MATERIAL", data["material_stack"])
    metric(draw, 542, "PROGRESS", f"{data['completed_count']} / 10", COLORS["green"])

    draw.rounded_rectangle((18, 620, 1262, 702), radius=10, fill=COLORS["panel"], outline=COLORS["line"])
    draw.text((34, 632), data["status"], font=font(17, True), fill=COLORS["white"])
    joint_text = "  ".join(
        f"J{index + 1} {float(row['position_deg']):+.0f} deg"
        for index, row in enumerate(data.get("joints", []))
    )
    draw.text(
        (34, 661),
        joint_text or "J1--  J2--  J3--  J4--  J5--  J6--",
        font=font(13, True),
        fill=COLORS["cyan"],
    )
    draw.text(
        (34, 684),
        f"Policy: {data['policy']} | UR10e fixed-link articulation | cuMotion collision-aware RMPflow",
        font=font(12),
        fill=COLORS["muted"],
    )
    return canvas


def main() -> None:
    args = parse_args()
    frame_paths = sorted(args.frames.glob("frame_*.png"))
    telemetry = json.loads((args.frames / "telemetry.json").read_text(encoding="utf-8"))
    if len(frame_paths) != len(telemetry):
        raise RuntimeError(f"frame/telemetry mismatch: {len(frame_paths)} != {len(telemetry)}")
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
