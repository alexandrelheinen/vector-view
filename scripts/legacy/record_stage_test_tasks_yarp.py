#!/usr/bin/env python3
"""Record StageTestTasks arm joint commands from YARP control ports (legacy stack).

Run this inside the pinned CoDyCo / Gazebo Classic container where YARP and the
ISIRWholeBodyController are available. It writes a vectorview-codyco-joint-log-v1
file that record_golden_trajectory.py can import.

Example:

    # Terminal 1: yarpserver, gazebo, ISIRWholeBodyController --sequence StageTestTasks
    python3 scripts/legacy/record_stage_test_tasks_yarp.py \
      --robot-prefix icubGazeboSim \
      --output /tmp/stage_test_tasks.codyco-log.json
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ARM_JOINT_ORDER = (
    "l_shoulder_pitch",
    "l_shoulder_roll",
    "l_shoulder_yaw",
    "l_elbow",
    "l_wrist_prosup",
    "l_wrist_pitch",
    "l_wrist_yaw",
    "r_shoulder_pitch",
    "r_shoulder_roll",
    "r_shoulder_yaw",
    "r_elbow",
    "r_wrist_prosup",
    "r_wrist_pitch",
    "r_wrist_yaw",
)


def read_arm_vector(port, joint_count: int = 7) -> list[float]:
    """Read the latest position command vector from a YARP control port."""
    bottle = port.read(False)
    if bottle is None:
        return []
    if bottle.size() < joint_count:
        return []
    return [float(bottle.get(i).asFloat64()) for i in range(joint_count)]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-prefix", default="icubGazeboSim")
    parser.add_argument("--left-arm-port", default=None)
    parser.add_argument("--right-arm-port", default=None)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--sequence", default="StageTestTasks")
    parser.add_argument("--container-image", default="")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    try:
        import yarp  # type: ignore[import-untyped]
    except ImportError:
        raise SystemExit(
            "YARP Python bindings are not installed in this environment. "
            "Run this script inside the legacy CoDyCo container."
        ) from None

    if not yarp.Network.checkNetwork():
        raise SystemExit("YARP network is not reachable; start yarpserver first.")

    left_port_name = args.left_arm_port or f"/{args.robot_prefix}/left_arm"
    right_port_name = args.right_arm_port or f"/{args.robot_prefix}/right_arm"

    left_input = yarp.BufferedPortVector()
    right_input = yarp.BufferedPortVector()
    left_input.open(left_port_name)
    right_input.open(right_port_name)

    dt = 1.0 / args.rate_hz
    samples: list[dict[str, object]] = []
    start = time.monotonic()
    print(f"Recording {args.duration:.1f}s from {left_port_name} and {right_port_name}")

    while time.monotonic() - start < args.duration:
        left = read_arm_vector(left_input)
        right = read_arm_vector(right_input)
        if len(left) == 7 and len(right) == 7:
            positions = left + right
            samples.append(
                {
                    "t": round(time.monotonic() - start, 6),
                    "positions": positions,
                }
            )
        time.sleep(dt)

    if not samples:
        raise SystemExit(
            "No arm samples were recorded. Confirm StageTestTasks is running and "
            f"the ports {left_port_name} / {right_port_name} are streaming."
        )

    document = {
        "format": "vectorview-codyco-joint-log-v1",
        "sequence": args.sequence,
        "rate_hz": args.rate_hz,
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "container_image": args.container_image,
        "robot_prefix": args.robot_prefix,
        "ports": {
            "left_arm": left_port_name,
            "right_arm": right_port_name,
        },
        "joint_order": list(ARM_JOINT_ORDER),
        "samples": samples,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2) + "\n")
    print(f"Wrote {args.output} ({len(samples)} samples)")


if __name__ == "__main__":
    main()
