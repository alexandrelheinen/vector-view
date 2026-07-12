"""Golden trajectory format and StageTestTasks grasp keyframe generation."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

SCHEMA_VERSION = 1
DEFAULT_RATE_HZ = 20.0
TOPIC_PREFIX = "/grasp_demo"

# Arm joint topics used by the release demo grasp sequence.
ARM_JOINTS = (
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


def topic_for_joint(joint: str) -> str:
    return f"{TOPIC_PREFIX}/{joint}"


@dataclass(frozen=True)
class Segment:
    """Linear interpolation of selected joints over a duration."""

    duration: float
    start: dict[str, float]
    end: dict[str, float]


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def interpolate_segment(segment: Segment, local_t: float) -> dict[str, float]:
    if segment.duration <= 0:
        return dict(segment.end)
    ratio = max(0.0, min(1.0, local_t / segment.duration))
    joints = set(segment.start) | set(segment.end)
    return {joint: lerp(segment.start[joint], segment.end[joint], ratio) for joint in joints}


def standing_arm_pose() -> dict[str, float]:
    return {
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


def reach_arm_pose() -> dict[str, float]:
    return {
        "l_shoulder_pitch": -0.95,
        "l_shoulder_roll": 1.35,
        "l_shoulder_yaw": 0.0,
        "l_elbow": 1.35,
        "l_wrist_prosup": 0.0,
        "l_wrist_pitch": -0.55,
        "l_wrist_yaw": 0.698,
        "r_shoulder_pitch": -0.95,
        "r_shoulder_roll": 1.35,
        "r_shoulder_yaw": 0.0,
        "r_elbow": 1.35,
        "r_wrist_prosup": 0.0,
        "r_wrist_pitch": -0.55,
        "r_wrist_yaw": 0.698,
    }


def press_arm_pose() -> dict[str, float]:
    return {
        "l_shoulder_pitch": -1.25,
        "l_shoulder_roll": 1.45,
        "l_shoulder_yaw": 0.12,
        "l_elbow": 0.55,
        "l_wrist_prosup": 0.0,
        "l_wrist_pitch": -0.35,
        "l_wrist_yaw": 0.698,
        "r_shoulder_pitch": -1.25,
        "r_shoulder_roll": 1.45,
        "r_shoulder_yaw": -0.12,
        "r_elbow": 0.55,
        "r_wrist_prosup": 0.0,
        "r_wrist_pitch": -0.35,
        "r_wrist_yaw": 0.698,
    }


def stage_test_tasks_segments() -> list[tuple[float, Segment | dict[str, float]]]:
    """Timeline segments mirroring scripts/animate_grasp.sh."""
    stand = standing_arm_pose()
    reach = reach_arm_pose()
    press = press_arm_pose()
    return [
        (1.0, stand),
        (3.0, Segment(3.0, stand, reach)),
        (2.5, Segment(2.5, reach, press)),
        (5.0, press),
        (3.0, Segment(3.0, press, stand)),
        (1.0, stand),
    ]


def sample_stage_test_tasks(rate_hz: float = DEFAULT_RATE_HZ) -> list[dict[str, object]]:
    dt = 1.0 / rate_hz
    timeline = stage_test_tasks_segments()
    frames: list[dict[str, object]] = []
    t = 0.0
    for duration, segment in timeline:
        if isinstance(segment, dict):
            end_t = t + duration
            while t < end_t - 1e-9:
                frames.append({"t": round(t, 6), "joints": dict(segment)})
                t += dt
        else:
            local = 0.0
            while local < segment.duration - 1e-9:
                joints = interpolate_segment(segment, local)
                frames.append({"t": round(t, 6), "joints": joints})
                t += dt
                local += dt
    if not frames or frames[-1]["t"] < round(t - dt, 6):
        frames.append({"t": round(max(0.0, t - dt), 6), "joints": standing_arm_pose()})
    return frames


def build_golden_document(
    *,
    source: str,
    source_detail: str,
    rate_hz: float = DEFAULT_RATE_HZ,
    notes: str | None = None,
) -> dict[str, object]:
    frames = sample_stage_test_tasks(rate_hz)
    return {
        "schema_version": SCHEMA_VERSION,
        "name": "stage_test_tasks",
        "source": source,
        "source_detail": source_detail,
        "topic_prefix": TOPIC_PREFIX,
        "joints": list(ARM_JOINTS),
        "rate_hz": rate_hz,
        "duration_seconds": frames[-1]["t"] if frames else 0.0,
        "notes": notes or "",
        "frames": frames,
    }


def joints_at_time(document: dict[str, object], time_seconds: float) -> dict[str, float]:
    frames = document["frames"]
    if not frames:
        raise ValueError("trajectory has no frames")
    if time_seconds <= float(frames[0]["t"]):
        return dict(frames[0]["joints"])
    for previous, current in zip(frames, frames[1:]):
        t0 = float(previous["t"])
        t1 = float(current["t"])
        if t0 <= time_seconds <= t1:
            if t1 <= t0:
                return dict(current["joints"])
            ratio = (time_seconds - t0) / (t1 - t0)
            joints = set(previous["joints"]) | set(current["joints"])
            return {
                joint: lerp(float(previous["joints"][joint]), float(current["joints"][joint]), ratio)
                for joint in joints
            }
    return dict(frames[-1]["joints"])


def sha256_document(document: dict[str, object]) -> str:
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(payload.encode()).hexdigest()


def load_golden(path: Path) -> dict[str, object]:
    document = json.loads(path.read_text())
    require_schema(document)
    return document


def require_schema(document: dict[str, object]) -> None:
    if document.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported schema_version: {document.get('schema_version')}")
    if "frames" not in document or not document["frames"]:
        raise ValueError("trajectory must contain at least one frame")
    for frame in document["frames"]:
        if "t" not in frame or "joints" not in frame:
            raise ValueError("each frame must contain t and joints")


def save_golden(path: Path, document: dict[str, object]) -> None:
    require_schema(document)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2) + "\n")


def frame_topics(frame: dict[str, object]) -> dict[str, float]:
    joints = frame["joints"]
    return {topic_for_joint(joint): float(value) for joint, value in joints.items()}


def compare_joint_sets(
    expected: dict[str, float],
    actual: dict[str, float],
    *,
    tolerance_rad: float,
    joints: Iterable[str] | None = None,
) -> dict[str, dict[str, float]]:
    selected = list(joints) if joints is not None else sorted(set(expected) | set(actual))
    deltas: dict[str, dict[str, float]] = {}
    for joint in selected:
        exp = float(expected[joint])
        got = float(actual.get(joint, actual.get(topic_for_joint(joint), 0.0)))
        delta = abs(exp - got)
        deltas[joint] = {"expected": exp, "actual": got, "abs_error": delta}
        if delta > tolerance_rad:
            raise ValueError(
                f"joint {joint} error {delta:.4f} rad exceeds tolerance {tolerance_rad:.4f} rad"
            )
    return deltas
