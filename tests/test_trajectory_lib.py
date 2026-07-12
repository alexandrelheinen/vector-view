#!/usr/bin/env python3
"""Unit tests for golden trajectory generation and CoDyCo log import."""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from codyco_log_import import build_golden_from_codyco_log  # noqa: E402
from trajectory_lib import (  # noqa: E402
    ARM_JOINTS,
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


class CodycoLogImportTest(unittest.TestCase):
    def test_import_codyco_v1_matches_hold_pose(self) -> None:
        path = ROOT / "tests/fixtures/stage_test_tasks.codyco-log.json"
        document = build_golden_from_codyco_log(path, rate_hz=20.0)
        self.assertEqual(document["source"], "codyco-log")
        joints = joints_at_time(document, 10.0)
        press = press_arm_pose()
        for joint in ARM_JOINTS:
            self.assertAlmostEqual(joints[joint], press[joint], places=2)

    def test_import_jsonl(self) -> None:
        path = ROOT / "tests/fixtures/stage_test_tasks.frames.jsonl"
        document = build_golden_from_codyco_log(path, rate_hz=20.0)
        self.assertGreater(len(document["frames"]), 20)

    def test_import_csv(self) -> None:
        path = ROOT / "tests/fixtures/stage_test_tasks.frames.csv"
        document = build_golden_from_codyco_log(path, rate_hz=20.0)
        self.assertGreater(len(document["frames"]), 20)

    def test_record_script_round_trip(self) -> None:
        bootstrap = ROOT / "assets/trajectories/stage_test_tasks.golden.json"
        output = ROOT / "build/test_imported.golden.json"
        output.parent.mkdir(parents=True, exist_ok=True)

        import subprocess

        subprocess.run(
            [
                "python3",
                str(ROOT / "scripts/record_golden_trajectory.py"),
                "--source",
                "codyco-log",
                "--codyco-log",
                str(ROOT / "tests/fixtures/stage_test_tasks.codyco-log.json"),
                "--output",
                str(output),
            ],
            check=True,
            cwd=ROOT,
        )
        imported = load_golden(output)
        expected = joints_at_time(load_golden(bootstrap), 10.0)
        actual = joints_at_time(imported, 10.0)
        for joint in ARM_JOINTS:
            self.assertAlmostEqual(actual[joint], expected[joint], places=2)


if __name__ == "__main__":
    unittest.main()
