#!/usr/bin/env python3
"""Unit tests for the modern grasp controller helpers."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from contact_monitor import ContactMonitor  # noqa: E402
from grasp_task_controller import PRESS_DELTAS, phase_plan  # noqa: E402
from trajectory_lib import press_arm_pose, reach_arm_pose  # noqa: E402


class ContactMonitorTest(unittest.TestCase):
    def test_detects_upper_box_contact_name(self) -> None:
        class Collision:
            def __init__(self, name: str) -> None:
                self.name = name

        class Contact:
            def __init__(self) -> None:
                self.collision2 = Collision("object::main::collision")

        class Message:
            contact = [Contact()]

        self.assertTrue(ContactMonitor._touches_upper_box(Message()))


class GraspControllerTest(unittest.TestCase):
    def test_phase_plan_covers_press_and_hold(self) -> None:
        names = [phase.name for phase in phase_plan()]
        self.assertEqual(names, ["STAND", "REACH", "PRESS", "HOLD", "RELEASE", "RETURN"])

    def test_press_deltas_move_shoulders_inward(self) -> None:
        self.assertLess(PRESS_DELTAS["l_shoulder_pitch"], 0.0)
        self.assertLess(PRESS_DELTAS["r_shoulder_pitch"], 0.0)

    def test_reach_and_press_poses_differ(self) -> None:
        reach = reach_arm_pose()
        press = press_arm_pose()
        self.assertNotEqual(reach["l_shoulder_pitch"], press["l_shoulder_pitch"])


if __name__ == "__main__":
    unittest.main()
