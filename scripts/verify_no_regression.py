#!/usr/bin/env python3
"""Fail unless the release demo proves real contacts, arrows, and force plots."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from trajectory_lib import (  # noqa: E402
    ARM_JOINTS,
    compare_joint_sets,
    joints_at_time,
    load_golden,
    sha256_document,
)


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
            y_coordinates = np.where(blue[:, cluster])[0]
            regions.append(
                {
                    "x_min": cluster[0],
                    "x_max": cluster[-1],
                    "y_min": int(y_coordinates.min()),
                    "y_max": int(y_coordinates.max()),
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


def frame_evidence(path: Path, *, require_two_arrows: bool) -> dict:
    frame = np.array(Image.open(path).convert("RGB"))
    require(frame.shape == (720, 1280, 3), f"unexpected frame dimensions: {frame.shape}")

    gray = frame.mean(axis=2)
    foreground = gray < 170
    y, x = np.where(foreground)
    require(len(x) > 10_000, f"{path.name} has too little robot / box foreground")
    bounds = {
        "x_min": int(x.min()),
        "x_max": int(x.max()),
        "y_min": int(y.min()),
        "y_max": int(y.max()),
    }
    require(bounds["x_min"] > 10, f"{path.name} is cropped on left edge: {bounds}")
    require(bounds["x_max"] < 1270, f"{path.name} is cropped on right edge: {bounds}")
    require(bounds["y_min"] > 10, f"{path.name} is cropped on top edge: {bounds}")
    require(bounds["y_max"] < 718, f"{path.name} is cropped on bottom edge: {bounds}")
    require(
        bounds["y_max"] - bounds["y_min"] >= 300,
        f"full robot and boxes are not visible in {path.name}: {bounds}",
    )

    color = frame.astype(np.int16)
    red = (
        (color[:, :, 0] > 180)
        & (color[:, :, 0] > color[:, :, 1] + 50)
        & (color[:, :, 0] > color[:, :, 2] + 50)
    )
    require(int(red.sum()) < 50, f"{path.name} contains suspicious red overlay pixels")

    evidence = {
        "width": 1280,
        "height": 720,
        "sha256": sha256(path),
        "foreground_bounds": bounds,
        "red_overlay_pixels": int(red.sum()),
    }
    if require_two_arrows:
        blue_pixels, arrow_regions = blue_clusters(frame)
        require(blue_pixels >= 800, f"only {blue_pixels} blue pixels in arrow frame")
        require(
            len(arrow_regions) >= 2,
            f"expected two separated blue arrow regions, found {arrow_regions}",
        )
        require(
            all(300 <= region["y_min"] and region["y_max"] <= 450 for region in arrow_regions),
            f"blue regions are not anchored in the fixed hand-height zone: {arrow_regions}",
        )
        evidence["blue_pixels"] = blue_pixels
        evidence["separated_arrow_regions"] = arrow_regions
    return evidence


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--comparison-frame", type=Path, required=True)
    parser.add_argument("--arrow-frame", type=Path, required=True)
    parser.add_argument("--left-contact", type=Path, required=True)
    parser.add_argument("--right-contact", type=Path, required=True)
    parser.add_argument("--left-plot", type=Path, required=True)
    parser.add_argument("--right-plot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--golden-trajectory", type=Path, required=True)
    parser.add_argument("--motion-snapshot", type=Path, required=True)
    parser.add_argument("--capture-time", type=float, required=True)
    parser.add_argument(
        "--joint-tolerance-rad",
        type=float,
        default=0.02,
        help="max allowed deviation between golden and replayed joint commands",
    )
    args = parser.parse_args()

    contacts = {}
    for hand, expected_y_sign, path in (
        ("left_hand", 1, args.left_contact),
        ("right_hand", -1, args.right_contact),
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
        position_match = re.search(
            r"position\s*\{\s*x:\s*([-0-9.eE]+)\s*"
            r"y:\s*([-0-9.eE]+)\s*z:\s*([-0-9.eE]+)",
            text,
        )
        require(position_match is not None, f"{hand} has no contact position")
        position = tuple(float(value) for value in position_match.groups())
        require(0.18 < position[0] < 0.22, f"{hand} contact x is not on upper box face: {position}")
        require(0.44 < position[2] < 0.50, f"{hand} contact z is not at hand height: {position}")
        require(
            expected_y_sign * position[1] > 0.15,
            f"{hand} is not on its expected side of upper box: {position}",
        )
        contacts[hand] = {
            "upper_box_contact": True,
            "position_m": {"x": position[0], "y": position[1], "z": position[2]},
            "message_bytes": len(text.encode()),
            "sha256": sha256(path),
        }

    comparison_frame = frame_evidence(args.comparison_frame, require_two_arrows=False)
    arrow_frame = frame_evidence(args.arrow_frame, require_two_arrows=True)

    plots = {
        "left_hand": plot_evidence(args.left_plot, "left hand"),
        "right_hand": plot_evidence(args.right_plot, "right hand"),
    }

    capture_script = Path(__file__).with_name("capture_comparison.sh").read_text()
    require(
        "overlay_force_arrows" not in capture_script,
        "capture pipeline still references the fake overlay script",
    )
    require(
        "replay_golden_trajectory.py" in capture_script,
        "capture pipeline must replay the golden trajectory",
    )

    golden = load_golden(args.golden_trajectory)
    golden_digest = sha256_document(golden)
    expected_joints = joints_at_time(golden, args.capture_time)
    snapshot = json.loads(args.motion_snapshot.read_text())
    require(
        abs(float(snapshot["capture_time_seconds"]) - args.capture_time) < 0.15,
        f"motion snapshot time mismatch: {snapshot}",
    )
    joint_deltas = compare_joint_sets(
        expected_joints,
        snapshot["joints"],
        tolerance_rad=args.joint_tolerance_rad,
        joints=ARM_JOINTS,
    )
    motion = {
        "golden_trajectory": str(args.golden_trajectory),
        "golden_sha256": golden_digest,
        "source": golden.get("source"),
        "capture_time_seconds": args.capture_time,
        "joint_tolerance_rad": args.joint_tolerance_rad,
        "joint_checks": joint_deltas,
        "motion_snapshot_sha256": sha256(args.motion_snapshot),
    }

    report = {
        "passed": True,
        "source": "unmodified /release_camera + /arrow_camera frames and /vectorview contact topics",
        "requirements": {
            requirement: True
            for requirement in (
                "V1",
                "V2",
                "V3",
                "V4",
                "V5",
                "S1",
                "S2",
                "S3",
                "D1",
                "D2",
                "D3",
                "M1",
                "M2",
                "M3",
                "I1",
                "I2",
                "I3",
            )
        },
        "motion": motion,
        "contacts": contacts,
        "comparison_frame": comparison_frame,
        "arrow_frame": arrow_frame,
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
