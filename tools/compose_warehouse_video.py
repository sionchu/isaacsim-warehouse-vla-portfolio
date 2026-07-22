from __future__ import annotations

import argparse
import json
from pathlib import Path

import av
import numpy as np
from PIL import Image, ImageDraw, ImageFont

WIDTH = 1280
HEIGHT = 720
MAIN_BOX = (20, 78, 890, 568)
PANEL_X = 920
PANEL_W = 340
COLORS = {
    "background": (17, 22, 29),
    "panel": (29, 37, 48),
    "line": (67, 82, 99),
    "white": (238, 244, 250),
    "muted": (158, 174, 190),
    "green": (48, 214, 125),
    "blue": (65, 145, 255),
    "orange": (255, 145, 55),
    "red": (255, 85, 95),
    "box": (198, 140, 62),
}


def font(size: int, bold: bool = False):
    name = "malgunbd.ttf" if bold else "malgun.ttf"
    path = Path("C:/Windows/Fonts") / name
    return ImageFont.truetype(str(path), size) if path.exists() else ImageFont.load_default()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--thumbnail", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=15)
    return parser.parse_args()


def map_point(x: float, y: float) -> tuple[int, int]:
    left, top, right, bottom = PANEL_X + 12, 105, PANEL_X + PANEL_W - 12, 382
    px = left + int((x + 8.0) / 14.5 * (right - left))
    py = bottom - int((y + 5.0) / 10.0 * (bottom - top))
    return px, py


def draw_map(draw: ImageDraw.ImageDraw, data: dict) -> None:
    left, top, right, bottom = PANEL_X + 12, 105, PANEL_X + PANEL_W - 12, 382
    draw.rounded_rectangle((left, top, right, bottom), radius=8, fill=(20, 27, 35), outline=COLORS["line"])
    for step in range(1, 7):
        x = left + step * (right - left) // 7
        draw.line((x, top, x, bottom), fill=(34, 45, 56), width=1)
    for step in range(1, 5):
        y = top + step * (bottom - top) // 5
        draw.line((left, y, right, y), fill=(34, 45, 56), width=1)

    for x0, x1, y0, y1 in ((-0.5, 1.2, -1.0, 1.0), (1.8, 2.8, 0.5, 2.0)):
        a = map_point(x0, y1)
        b = map_point(x1, y0)
        draw.rectangle((*a, *b), fill=(91, 72, 55), outline=(145, 109, 72), width=2)

    for label, position, color in (
        ("A", (-6.0, 0.0), COLORS["blue"]),
        ("B", (4.0, 2.6), COLORS["green"]),
        ("C", (4.0, -2.6), COLORS["orange"]),
    ):
        x, y = map_point(*position)
        draw.ellipse((x - 13, y - 13, x + 13, y + 13), fill=color)
        draw.text((x, y - 1), label, font=font(16, True), fill=(10, 15, 20), anchor="mm")

    route = data.get("route", [])
    if len(route) >= 2:
        draw.line([map_point(*point) for point in route], fill=COLORS["green"], width=4, joint="curve")
    robot_x, robot_y = map_point(*data["position"])
    draw.ellipse((robot_x - 9, robot_y - 9, robot_x + 9, robot_y + 9), fill=COLORS["white"], outline=COLORS["blue"], width=3)
    draw.line((robot_x, robot_y, robot_x + 14, robot_y), fill=COLORS["blue"], width=3)


def draw_occupancy(draw: ImageDraw.ImageDraw, data: dict) -> None:
    draw.text((PANEL_X + 14, 401), "KANBAN OCCUPANCY", font=font(16, True), fill=COLORS["white"])
    occupancy = data["occupancy"]
    selected = data.get("selected_slot", "")
    for row, rack in enumerate(("B", "C")):
        y = 434 + row * 45
        draw.text((PANEL_X + 14, y + 12), rack, font=font(18, True), fill=COLORS["green"] if rack == "B" else COLORS["orange"], anchor="lm")
        for level in range(1, 6):
            slot = f"{rack}{level}"
            x = PANEL_X + 48 + (level - 1) * 55
            fill = COLORS["box"] if occupancy[slot] else (43, 55, 68)
            outline = COLORS["white"] if slot == selected else COLORS["line"]
            draw.rounded_rectangle((x, y, x + 44, y + 31), radius=5, fill=fill, outline=outline, width=2)
            draw.text((x + 22, y + 16), str(level), font=font(14, True), fill=COLORS["white"], anchor="mm")


def compose_frame(raw: Image.Image, data: dict, index: int, fps: int) -> Image.Image:
    canvas = Image.new("RGB", (WIDTH, HEIGHT), COLORS["background"])
    draw = ImageDraw.Draw(canvas)
    draw.text((20, 18), "ISAAC SIM WAREHOUSE VLA TRIAL", font=font(30, True), fill=COLORS["white"])
    draw.text((1260, 28), f"T+{index / fps:05.1f}s", font=font(18), fill=COLORS["muted"], anchor="ra")

    main = raw.convert("RGB").resize((MAIN_BOX[2] - MAIN_BOX[0], MAIN_BOX[3] - MAIN_BOX[1]), Image.Resampling.LANCZOS)
    canvas.paste(main, (MAIN_BOX[0], MAIN_BOX[1]))
    draw.rounded_rectangle(MAIN_BOX, radius=8, outline=COLORS["line"], width=2)
    draw.text((32, 91), "ISAAC SIM VIEW", font=font(15, True), fill=COLORS["white"])

    draw.rounded_rectangle((PANEL_X, 78, PANEL_X + PANEL_W, 700), radius=10, fill=COLORS["panel"], outline=COLORS["line"])
    draw.text((PANEL_X + 14, 88), "SLAM / LIDAR VIEW", font=font(16, True), fill=COLORS["white"])
    draw_map(draw, data)
    draw_occupancy(draw, data)

    state_color = COLORS["green"] if data["state"] == "IDLE" else COLORS["blue"]
    draw.rounded_rectangle((20, 585, 890, 700), radius=10, fill=COLORS["panel"], outline=COLORS["line"])
    draw.text((36, 600), data["mission"], font=font(23, True), fill=COLORS["white"])
    draw.text((36, 637), data["status"], font=font(18), fill=COLORS["muted"])
    draw.rounded_rectangle((733, 600, 870, 641), radius=8, fill=state_color)
    draw.text((801, 620), data["state"], font=font(16, True), fill=(12, 18, 23), anchor="mm")
    draw.text((36, 674), f"Policy: {data['policy']}  |  Safe route: A* / Nav2 architecture", font=font(15), fill=COLORS["muted"])
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
    thumbnail_index = min(max(len(frame_paths) // 3, 0), len(frame_paths) - 1)

    for index, (path, data) in enumerate(zip(frame_paths, telemetry)):
        composed = compose_frame(Image.open(path), data, index, args.fps)
        if index == thumbnail_index:
            composed.save(args.thumbnail, quality=92)
        video_frame = av.VideoFrame.from_ndarray(np.asarray(composed), format="rgb24")
        for packet in stream.encode(video_frame):
            container.mux(packet)
        if index % args.fps == 0:
            print(f"encode frame={index:03d}", flush=True)

    for packet in stream.encode():
        container.mux(packet)
    container.close()
    print(f"video={args.output}")
    print(f"thumbnail={args.thumbnail}")


if __name__ == "__main__":
    main()
