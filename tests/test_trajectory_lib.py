#!/usr/bin/env python3
"""Unit tests for golden trajectory generation (no Gazebo required)."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from trajectory_lib import (  # noqa: E402
    build_golden_document,
    joints_at_time,
    load_golden,
    press_arm_pose,
    sample_stage_test_tasks,
    sha256_document,
)


class TrajectoryLibTest(unittest.TestCase):
    def test_stage_test_tasks_duration(self) -> None:
        frames = sample_stage_test_tasks()
        self.assertGreater(len(frames), 200)
        self.assertAlmostEqual(frames[-1]["t"], 15.45, places=2)

    def test_hold_phase_matches_press_pose(self) -> None:
        joints = joints_at_time(
            {"frames": sample_stage_test_tasks()},
            10.0,
        )
        press = press_arm_pose()
        for joint, value in press.items():
            self.assertAlmostEqual(joints[joint], value, places=3)

    def test_committed_golden_file_loads(self) -> None:
        path = ROOT / "assets/trajectories/stage_test_tasks.golden.json"
        document = load_golden(path)
        self.assertEqual(document["schema_version"], 1)
        self.assertEqual(document["name"], "stage_test_tasks")
        digest = sha256_document(document)
        on_disk = json.loads(path.read_text())
        self.assertEqual(digest, sha256_document(on_disk))

    def test_build_is_deterministic(self) -> None:
        first = build_golden_document(source="test", source_detail="test")
        second = build_golden_document(source="test", source_detail="test")
        self.assertEqual(sha256_document(first), sha256_document(second))


if __name__ == "__main__":
    unittest.main()
