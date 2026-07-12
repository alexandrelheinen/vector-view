#!/usr/bin/env python3
"""Capture a camera frame at a specific sim-time offset for pose tuning."""

from __future__ import annotations

import argparse
import os
import subprocess
import time
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--warmup", type=float, default=12.0)
    parser.add_argument("--capture-after", type=float, default=10.0)
    parser.add_argument("--world", default="assets/worlds/release_demo.world")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env["VECTOR_VIEW"] = str(root)
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{root / 'build'}:{env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')}"
    env["GZ_SIM_RESOURCE_PATH"] = f"{root / 'assets/models'}:{env.get('GZ_SIM_RESOURCE_PATH', '')}"
    env["GZ_SIM_USER_PATH"] = f"{root / 'assets/worlds'}:{env.get('GZ_SIM_USER_PATH', '')}"

    subprocess.run(
        ["pkill", "-9", "-f", r"^gz sim( |$)"],
        check=False,
        capture_output=True,
    )
    time.sleep(1)

    world = root / args.world
    log = open("/tmp/gz_preview.log", "w")
    proc = subprocess.Popen(
        ["gz", "sim", "-s", "-r", "--headless-rendering", str(world)],
        env={**env, "DISPLAY": ""},
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    time.sleep(args.warmup)

    anim = subprocess.Popen(
        [str(root / "scripts/animate_grasp.sh")],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(args.capture_after)

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["python3", str(root / "scripts/save_camera_frame.py"), "--output", str(out), "--timeout", "15"],
        check=True,
        env=env,
    )

    anim.terminate()
    try:
        anim.wait(timeout=5)
    except subprocess.TimeoutExpired:
        anim.kill()
    proc.kill()
    subprocess.run(
        ["pkill", "-9", "-f", r"^gz sim( |$)"],
        check=False,
        capture_output=True,
    )
    print(f"saved {out}")


if __name__ == "__main__":
    main()
