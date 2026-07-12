#!/usr/bin/env python3
"""Record or regenerate the StageTestTasks golden joint trajectory."""

from __future__ import annotations

import argparse
from pathlib import Path

from trajectory_lib import (
    DEFAULT_RATE_HZ,
    build_golden_document,
    save_golden,
    sha256_document,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("assets/trajectories/stage_test_tasks.golden.json"),
        help="golden trajectory output path",
    )
    parser.add_argument(
        "--source",
        choices=("animate-grasp-keyframes", "codyco-log"),
        default="animate-grasp-keyframes",
        help="trajectory provenance",
    )
    parser.add_argument(
        "--codyco-log",
        type=Path,
        help="future CoDyCo joint-command log to import (not yet implemented)",
    )
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    args = parser.parse_args()

    if args.source == "codyco-log":
        raise SystemExit(
            "codyco-log import is not implemented yet; record StageTestTasks in the "
            "legacy CoDyCo container and open a follow-up to add the parser."
        )

    document = build_golden_document(
        source="animate-grasp-keyframes",
        source_detail="Keyframe port of scripts/animate_grasp.sh until CoDyCo StageTestTasks is recorded",
        rate_hz=args.rate_hz,
        notes=(
            "Replace this file by re-running record_golden_trajectory.py --source codyco-log "
            "once a canonical CoDyCo StageTestTasks recording exists."
        ),
    )
    save_golden(args.output, document)
    digest = sha256_document(document)
    print(f"Wrote {args.output}")
    print(f"frames={len(document['frames'])} duration={document['duration_seconds']:.3f}s sha256={digest}")


if __name__ == "__main__":
    main()
