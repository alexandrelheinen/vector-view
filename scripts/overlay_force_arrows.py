#!/usr/bin/env python3
"""Draw contact-force arrows onto a captured Gazebo camera frame."""

from __future__ import annotations

import argparse
import math
import re
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw


def sample_force(topic: str) -> tuple[float, float, float]:
    proc = subprocess.run(
        ["timeout", "3", "gz", "topic", "-e", "-t", topic, "-n", "1"],
        capture_output=True,
        text=True,
        check=False,
    )
    fx = fy = fz = 0.0
    for block in re.findall(r"body_1_wrench\s*\{[^}]*force\s*\{([^}]*)\}", proc.stdout, flags=re.S):
        fx += float(m.group(1)) if (m := re.search(r"x:\s*([-0-9.eE]+)", block)) else 0.0
        fy += float(m.group(1)) if (m := re.search(r"y:\s*([-0-9.eE]+)", block)) else 0.0
        fz += float(m.group(1)) if (m := re.search(r"z:\s*([-0-9.eE]+)", block)) else 0.0
    return fx, fy, fz


def draw_arrow(
    draw: ImageDraw.ImageDraw,
    origin: tuple[float, float],
    force: tuple[float, float, float],
    screen_dir: tuple[float, float],
    color: str,
) -> None:
    magnitude = math.sqrt(force[0] ** 2 + force[1] ** 2 + force[2] ** 2)
    if magnitude < 1.0:
        return

    length = min(90.0, max(35.0, magnitude * 1.4))
    dx, dy = screen_dir
    norm = math.hypot(dx, dy) or 1.0
    dx, dy = dx / norm, dy / norm
    tip = (origin[0] + dx * length, origin[1] + dy * length)
    draw.line(origin + tip, fill=color, width=6)

    head = 14.0
    angle = math.atan2(dy, dx)
    left = (
        tip[0] - head * math.cos(angle - math.pi / 7),
        tip[1] - head * math.sin(angle - math.pi / 7),
    )
    right = (
        tip[0] - head * math.cos(angle + math.pi / 7),
        tip[1] - head * math.sin(angle + math.pi / 7),
    )
    draw.polygon([tip, left, right], fill=color)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--l-topic", default="/vectorview/iCub_fixed/l_hand")
    parser.add_argument("--r-topic", default="/vectorview/iCub_fixed/r_hand")
    args = parser.parse_args()

    image = Image.open(args.image).convert("RGB")
    draw = ImageDraw.Draw(image)

    # Origins tuned for release_camera pose in release_demo.world (1280x720).
    l_origin = (292, 392)
    r_origin = (292, 452)
    l_force = sample_force(args.l_topic)
    r_force = sample_force(args.r_topic)

    # Screen-left / screen-right for lateral contact on the upper box.
    draw_arrow(draw, l_origin, l_force, (-1.0, 0.05), "#e53935")
    draw_arrow(draw, r_origin, r_force, (-1.0, -0.05), "#e53935")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output)
    print(f"overlay saved to {output}")


if __name__ == "__main__":
    main()
