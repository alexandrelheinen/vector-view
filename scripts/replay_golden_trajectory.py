#!/usr/bin/env python3
"""Replay a golden joint trajectory through /grasp_demo JointPositionController topics."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node

from trajectory_lib import frame_topics, load_golden, topic_for_joint


def publish_frame(node: Node, publishers: dict[str, object], topics: dict[str, float]) -> None:
    message = Double()
    for topic, value in topics.items():
        publisher = publishers[topic]
        message.data = value
        publisher.publish(message)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trajectory",
        type=Path,
        default=Path("assets/trajectories/stage_test_tasks.golden.json"),
    )
    parser.add_argument(
        "--snapshot-file",
        type=Path,
        help="write the active joint command snapshot on exit (used by capture_comparison.sh)",
    )
    parser.add_argument(
        "--snapshot-at",
        type=float,
        help="also write a snapshot when replay reaches this time (seconds)",
    )
    parser.add_argument("--start-offset", type=float, default=0.0, help="skip initial seconds")
    args = parser.parse_args()

    document = load_golden(args.trajectory)
    frames = document["frames"]
    rate_hz = float(document.get("rate_hz", 20.0))
    dt = 1.0 / rate_hz

    node = Node()
    publishers: dict[str, object] = {}
    for joint in document.get("joints", []):
        topic = topic_for_joint(str(joint))
        publishers[topic] = node.advertise(topic, Double)

    start = time.monotonic()
    last_snapshot: dict[str, float] | None = None
    snapshot_at_written = False

    for index, frame in enumerate(frames):
        frame_time = float(frame["t"])
        if frame_time < args.start_offset:
            continue
        topics = frame_topics(frame)
        publish_frame(node, publishers, topics)
        last_snapshot = dict(frame["joints"])

        if args.snapshot_at is not None and not snapshot_at_written:
            if frame_time >= args.snapshot_at - 1e-6:
                write_snapshot(args.snapshot_file, frame_time, last_snapshot, document)
                snapshot_at_written = True

        if index + 1 < len(frames):
            next_time = float(frames[index + 1]["t"])
            sleep_for = max(0.0, next_time - frame_time)
            if sleep_for > 0:
                time.sleep(sleep_for)

    if args.snapshot_file is not None and last_snapshot is not None and not snapshot_at_written:
        write_snapshot(
            args.snapshot_file,
            float(frames[-1]["t"]),
            last_snapshot,
            document,
        )

    elapsed = time.monotonic() - start
    print(
        f"Replayed {len(frames)} frames from {args.trajectory} in {elapsed:.2f}s real time",
        file=sys.stderr,
    )


def write_snapshot(
    path: Path | None,
    capture_time: float,
    joints: dict[str, float],
    document: dict[str, object],
) -> None:
    if path is None:
        return
    payload = {
        "capture_time_seconds": capture_time,
        "trajectory": str(document.get("name", "unknown")),
        "joints": joints,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


if __name__ == "__main__":
    main()
