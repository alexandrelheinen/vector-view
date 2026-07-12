#!/usr/bin/env python3
"""Fail unless the release demo proves real contacts, arrows, and force plots."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(f"NO-REGRESSION FAILURE: {message}")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def blue_clusters(image: np.ndarray) -> tuple[int, list[dict[str, int]]]:
    """Find substantial, horizontally separated blue regions."""
    color = image.astype(np.int16)
    blue = (
        (color[:, :, 2] > 150)
        & (color[:, :, 2] > color[:, :, 0] + 40)
        & (color[:, :, 2] > color[:, :, 1] + 30)
    )
    columns = np.where(blue.sum(axis=0) > 2)[0]
    clusters: list[list[int]] = []
    for column in columns:
        if not clusters or column - clusters[-1][-1] > 5:
            clusters.append([int(column)])
        else:
            clusters[-1].append(int(column))

    regions = []
    for cluster in clusters:
        pixels = int(blue[:, cluster].sum())
        if pixels >= 200:
            regions.append(
                {
                    "x_min": cluster[0],
                    "x_max": cluster[-1],
                    "pixels": pixels,
                }
            )
    return int(blue.sum()), regions


def plot_evidence(path: Path, hand: str) -> dict:
    data = json.loads(path.read_text())
    require(data["samples"] >= 200, f"{hand} plot has too few samples: {data}")
    require(data["messages"] >= 20, f"{hand} received too few contact messages: {data}")
    require(
        data["nonzero_samples"] >= 10,
        f"{hand} plot has no sustained non-zero force: {data}",
    )
    require(
        data["max_force_newtons"] >= 1.0,
        f"{hand} peak force is below 1 N: {data}",
    )
    require(
        data["duration_seconds"] >= 10.0,
        f"{hand} trace does not cover the full interaction: {data}",
    )
    return data


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frame", type=Path, required=True)
    parser.add_argument("--left-contact", type=Path, required=True)
    parser.add_argument("--right-contact", type=Path, required=True)
    parser.add_argument("--left-plot", type=Path, required=True)
    parser.add_argument("--right-plot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    contacts = {}
    for hand, path in (
        ("left_hand", args.left_contact),
        ("right_hand", args.right_contact),
    ):
        text = path.read_text()
        require(
            "object::main::collision" in text,
            f"{hand} is not touching object::main::collision",
        )
        require(
            f"::{hand.removesuffix('_hand')[0]}_hand::" in text,
            f"{hand} contact message does not name its hand collision",
        )
        contacts[hand] = {
            "upper_box_contact": True,
            "message_bytes": len(text.encode()),
            "sha256": sha256(path),
        }

    frame = np.array(Image.open(args.frame).convert("RGB"))
    require(frame.shape == (720, 1280, 3), f"unexpected frame dimensions: {frame.shape}")

    blue_pixels, arrow_regions = blue_clusters(frame)
    require(blue_pixels >= 800, f"only {blue_pixels} blue pixels in raw camera frame")
    require(
        len(arrow_regions) >= 2,
        f"expected two separated blue arrow regions, found {arrow_regions}",
    )

    gray = frame.mean(axis=2)
    foreground = gray < 170
    y, x = np.where(foreground)
    require(len(x) > 10_000, "camera frame has too little robot / box foreground")
    bounds = {
        "x_min": int(x.min()),
        "x_max": int(x.max()),
        "y_min": int(y.min()),
        "y_max": int(y.max()),
    }
    require(bounds["x_min"] > 10, f"scene is cropped on left edge: {bounds}")
    require(bounds["x_max"] < 1270, f"scene is cropped on right edge: {bounds}")
    require(bounds["y_min"] > 10, f"scene is cropped on top edge: {bounds}")
    require(bounds["y_max"] < 718, f"scene is cropped on bottom edge: {bounds}")
    require(
        bounds["y_max"] - bounds["y_min"] >= 300,
        f"full robot and boxes are not visible: {bounds}",
    )

    color = frame.astype(np.int16)
    red = (
        (color[:, :, 0] > 180)
        & (color[:, :, 0] > color[:, :, 1] + 50)
        & (color[:, :, 0] > color[:, :, 2] + 50)
    )
    require(int(red.sum()) < 50, "raw camera frame contains suspicious red overlay pixels")

    plots = {
        "left_hand": plot_evidence(args.left_plot, "left hand"),
        "right_hand": plot_evidence(args.right_plot, "right hand"),
    }

    capture_script = Path(__file__).with_name("capture_comparison.sh").read_text()
    require(
        "overlay_force_arrows" not in capture_script,
        "capture pipeline still references the fake overlay script",
    )

    report = {
        "passed": True,
        "source": "unmodified /release_camera frame and /vectorview contact topics",
        "contacts": contacts,
        "raw_frame": {
            "width": 1280,
            "height": 720,
            "sha256": sha256(args.frame),
            "foreground_bounds": bounds,
            "blue_pixels": blue_pixels,
            "separated_arrow_regions": arrow_regions,
            "red_overlay_pixels": int(red.sum()),
        },
        "plots": plots,
        "capture_pipeline": {
            "overlay_script_referenced": False,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("NO-REGRESSION PASS")


if __name__ == "__main__":
    main()
