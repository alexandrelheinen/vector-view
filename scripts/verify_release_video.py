#!/usr/bin/env python3
"""Fail unless a release grasp video shows a full-duration, in-sim demo."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image

MIN_DURATION_SECONDS = 10.0
MIN_FRAME_COUNT = 200
MIN_FILE_BYTES = 100_000
MIN_HOLD_BLUE_PIXELS = 80


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(f"RELEASE-VIDEO FAILURE: {message}")


def ffprobe_field(path: Path, field: str) -> str:
    result = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-show_entries",
            field,
            "-of",
            "default=noprint_wrappers=1:nokey=1",
            str(path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    require(result.returncode == 0, f"ffprobe failed for {path}: {result.stderr.strip()}")
    return result.stdout.strip()


def extract_frame(path: Path, timestamp_seconds: float, output: Path) -> None:
    result = subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-ss",
            f"{timestamp_seconds:.3f}",
            "-i",
            str(path),
            "-frames:v",
            "1",
            str(output),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    require(result.returncode == 0, f"ffmpeg frame extract failed: {result.stderr.strip()}")
    require(output.is_file() and output.stat().st_size > 0, "extracted frame is empty")


def blue_pixel_count(image: np.ndarray) -> int:
    color = image.astype(np.int16)
    blue = (
        (color[:, :, 2] > 150)
        & (color[:, :, 2] > color[:, :, 0] + 40)
        & (color[:, :, 2] > color[:, :, 1] + 30)
    )
    return int(blue.sum())


def foreground_bounds(image: np.ndarray) -> dict[str, int]:
    gray = image.mean(axis=2)
    mask = gray < 235
    require(mask.any(), "frame is blank; robot and boxes are not visible")
    ys, xs = np.where(mask)
    return {
        "x_min": int(xs.min()),
        "x_max": int(xs.max()),
        "y_min": int(ys.min()),
        "y_max": int(ys.max()),
    }


def verify(path: Path) -> dict:
    require(path.is_file(), f"missing video: {path}")
    size_bytes = path.stat().st_size
    require(size_bytes >= MIN_FILE_BYTES, f"video too small ({size_bytes} bytes)")

    duration = float(ffprobe_field(path, "format=duration"))
    frames = int(float(ffprobe_field(path, "stream=nb_frames")))
    require(duration >= MIN_DURATION_SECONDS, f"duration {duration:.3f}s < {MIN_DURATION_SECONDS}s")
    require(frames >= MIN_FRAME_COUNT, f"frame count {frames} < {MIN_FRAME_COUNT}")

    hold_time = max(2.0, duration * 0.55)
    with tempfile.TemporaryDirectory() as tmp_dir:
        hold_frame = Path(tmp_dir) / "hold.png"
        extract_frame(path, hold_time, hold_frame)
        image = np.asarray(Image.open(hold_frame).convert("RGB"))
        bounds = foreground_bounds(image)
        height = bounds["y_max"] - bounds["y_min"] + 1
        width = bounds["x_max"] - bounds["x_min"] + 1
        require(height >= 300, f"scene height {height}px is too cropped for full robot")
        require(width >= 250, f"scene width {width}px is too cropped for robot and boxes")

        blue_pixels = blue_pixel_count(image)
        require(
            blue_pixels >= MIN_HOLD_BLUE_PIXELS,
            f"expected contact-force arrows during HOLD; found {blue_pixels} blue pixels",
        )

    return {
        "passed": True,
        "path": str(path),
        "size_bytes": size_bytes,
        "duration_seconds": duration,
        "frame_count": frames,
        "hold_sample_seconds": hold_time,
        "hold_blue_pixels": blue_pixels,
        "hold_foreground_bounds": bounds,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    report = verify(args.video)
    if args.report is not None:
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print("RELEASE-VIDEO PASS")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
