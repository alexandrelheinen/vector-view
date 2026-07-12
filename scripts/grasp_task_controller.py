"""Task-space grasp controller for the release demo on Gazebo Harmonic.

Uses Gazebo Harmonic JointPositionController topics plus live contact feedback.
No CoDyCo, YARP, or recorded trajectories are required.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node

from contact_monitor import ContactMonitor
from trajectory_lib import ARM_JOINTS, press_arm_pose, reach_arm_pose, topic_for_joint

STAND_JOINTS = {
    "l_shoulder_pitch": -0.52,
    "l_shoulder_roll": 0.52,
    "l_shoulder_yaw": 0.0,
    "l_elbow": 0.785,
    "l_wrist_prosup": 0.0,
    "l_wrist_pitch": 0.0,
    "l_wrist_yaw": 0.698,
    "r_shoulder_pitch": -0.52,
    "r_shoulder_roll": 0.52,
    "r_shoulder_yaw": 0.0,
    "r_elbow": 0.785,
    "r_wrist_prosup": 0.0,
    "r_wrist_pitch": 0.0,
    "r_wrist_yaw": 0.698,
}

JOINT_LIMITS = {
    "l_shoulder_pitch": (-1.65806, 0.0872665),
    "l_shoulder_roll": (0.0, 2.80649),
    "l_shoulder_yaw": (-0.645772, 1.74533),
    "l_elbow": (0.0959931, 1.85005),
    "l_wrist_prosup": (-0.872665, 0.872665),
    "l_wrist_pitch": (-1.13446, 0.174533),
    "l_wrist_yaw": (-0.436332, 0.436332),
    "r_shoulder_pitch": (-1.65806, 0.0872665),
    "r_shoulder_roll": (0.0, 2.80649),
    "r_shoulder_yaw": (-0.645772, 1.74533),
    "r_elbow": (0.0959931, 1.85005),
    "r_wrist_prosup": (-0.872665, 0.872665),
    "r_wrist_pitch": (-1.13446, 0.174533),
    "r_wrist_yaw": (-0.436332, 0.436332),
}

PRESS_DELTAS = {
    "l_shoulder_pitch": -0.04,
    "l_shoulder_roll": 0.03,
    "l_shoulder_yaw": 0.03,
    "l_elbow": -0.05,
    "l_wrist_pitch": 0.03,
    "r_shoulder_pitch": -0.04,
    "r_shoulder_roll": 0.03,
    "r_shoulder_yaw": -0.03,
    "r_elbow": -0.05,
    "r_wrist_pitch": 0.03,
}


@dataclass(frozen=True)
class Phase:
    name: str
    duration: float


def clamp_joint(joint: str, value: float) -> float:
    low, high = JOINT_LIMITS[joint]
    return float(max(low, min(high, value)))


def phase_plan() -> list[Phase]:
    return [
        Phase("STAND", 1.0),
        Phase("REACH", 3.0),
        Phase("PRESS", 2.5),
        Phase("HOLD", 5.0),
        Phase("RELEASE", 3.0),
        Phase("RETURN", 1.0),
    ]


class JointPublisher:
    def __init__(self) -> None:
        self._node = Node()
        self._publishers = {
            topic_for_joint(joint): self._node.advertise(topic_for_joint(joint), Double)
            for joint in ARM_JOINTS
        }
        self._message = Double()
        self._current = dict(STAND_JOINTS)

    def publish(self, joints: dict[str, float]) -> None:
        self._current = dict(joints)
        for joint, value in joints.items():
            self._message.data = float(value)
            self._publishers[topic_for_joint(joint)].publish(self._message)

    @property
    def current(self) -> dict[str, float]:
        return dict(self._current)


class GraspTaskController:
    def __init__(self, publisher: JointPublisher, contacts: ContactMonitor) -> None:
        self._publisher = publisher
        self._contacts = contacts
        self._active_phase = "INIT"
        self._press_pose = dict(press_arm_pose())

    def interpolate(
        self,
        start: dict[str, float],
        end: dict[str, float],
        duration: float,
        rate_hz: float,
    ) -> dict[str, float]:
        steps = max(1, int(duration * rate_hz))
        dt = duration / steps
        joints = dict(start)
        for step in range(1, steps + 1):
            ratio = step / steps
            for joint in ARM_JOINTS:
                joints[joint] = start[joint] + (end[joint] - start[joint]) * ratio
            self._publisher.publish(joints)
            time.sleep(dt)
        return joints

    def hold(self, joints: dict[str, float], duration: float, rate_hz: float) -> None:
        dt = 1.0 / rate_hz
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self._publisher.publish(joints)
            time.sleep(dt)

    def press_until_contact(self, start: dict[str, float], duration: float, rate_hz: float) -> dict[str, float]:
        """Close the loop on upper-box contact instead of replaying joint logs."""
        dt = 1.0 / rate_hz
        end = time.monotonic() + duration
        joints = dict(start)
        step_index = 0
        while time.monotonic() < end:
            if not self._contacts.both_on_box():
                for joint, delta in PRESS_DELTAS.items():
                    joints[joint] = clamp_joint(joint, joints.get(joint, 0.0) + delta)
                step_index += 1
            self._publisher.publish(joints)
            time.sleep(dt)
        self._press_pose = dict(joints)
        return joints

    def release_from_contact(self, start: dict[str, float], duration: float, rate_hz: float) -> dict[str, float]:
        return self.interpolate(start, STAND_JOINTS, duration, rate_hz)

    def run(
        self,
        *,
        rate_hz: float = 20.0,
        snapshot_at: float | None = None,
        snapshot_file: Path | None = None,
    ) -> None:
        current = dict(STAND_JOINTS)
        self._publisher.publish(current)
        elapsed = 0.0
        snapshot_written = False

        for phase in phase_plan():
            self._active_phase = phase.name
            print(f"Grasp controller: phase {phase.name}", file=sys.stderr)

            if phase.name == "STAND":
                self.hold(current, phase.duration, rate_hz)
            elif phase.name == "REACH":
                current = self.interpolate(current, reach_arm_pose(), phase.duration, rate_hz)
            elif phase.name == "PRESS":
                current = self.press_until_contact(current, phase.duration, rate_hz)
            elif phase.name == "HOLD":
                elapsed, snapshot_written = self.hold_with_snapshot(
                    current,
                    phase.duration,
                    rate_hz,
                    elapsed,
                    snapshot_at,
                    snapshot_file,
                    snapshot_written,
                )
            elif phase.name == "RELEASE":
                current = self.release_from_contact(current, phase.duration, rate_hz)
            else:
                current = self.interpolate(current, STAND_JOINTS, phase.duration, rate_hz)

            if phase.name != "HOLD":
                elapsed += phase.duration

        if snapshot_file is not None and not snapshot_written:
            write_snapshot(snapshot_file, self._active_phase, self._contacts, self._publisher.current, elapsed)

    def hold_with_snapshot(
        self,
        joints: dict[str, float],
        duration: float,
        rate_hz: float,
        elapsed: float,
        snapshot_at: float | None,
        snapshot_file: Path | None,
        snapshot_written: bool,
    ) -> tuple[float, bool]:
        dt = 1.0 / rate_hz
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self._publisher.publish(joints)
            time.sleep(dt)
            elapsed += dt
            if (
                snapshot_at is not None
                and not snapshot_written
                and elapsed >= snapshot_at - 1e-6
            ):
                write_snapshot(snapshot_file, "HOLD", self._contacts, self._publisher.current, elapsed)
                snapshot_written = True
        return elapsed, snapshot_written


def write_snapshot(
    path: Path | None,
    phase: str,
    contacts: ContactMonitor,
    joints: dict[str, float],
    elapsed: float,
) -> None:
    if path is None:
        return
    payload = {
        "controller": "grasp_task_controller",
        "capture_time_seconds": elapsed,
        "phase": phase,
        "left_on_box": contacts.left_on_box,
        "right_on_box": contacts.right_on_box,
        "both_on_box": contacts.both_on_box(),
        "joints": joints,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--snapshot-at", type=float)
    parser.add_argument("--snapshot-file", type=Path)
    args = parser.parse_args()

    contacts = ContactMonitor(
        "/vectorview/iCub_fixed/l_hand",
        "/vectorview/iCub_fixed/r_hand",
    )
    controller = GraspTaskController(JointPublisher(), contacts)
    controller.run(
        rate_hz=args.rate_hz,
        snapshot_at=args.snapshot_at,
        snapshot_file=args.snapshot_file,
    )


if __name__ == "__main__":
    main()
