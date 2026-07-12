"""Import CoDyCo / YARP joint-command logs into golden trajectory documents."""

from __future__ import annotations

import csv
import json
from pathlib import Path

from trajectory_lib import (
    ARM_JOINTS,
    DEFAULT_RATE_HZ,
    SCHEMA_VERSION,
    TOPIC_PREFIX,
    lerp,
    require_schema,
)

CODYCO_LOG_FORMAT_V1 = "vectorview-codyco-joint-log-v1"


def load_raw_frames(path: Path) -> tuple[list[dict[str, object]], dict[str, object]]:
    """Load irregular-time joint samples from a supported log file."""
    text = path.read_text()
    suffix = path.suffix.lower()

    if suffix == ".jsonl":
        return _load_jsonl(text), {"input_format": "jsonl"}

    document = json.loads(text)
    if isinstance(document, dict) and document.get("schema_version") == SCHEMA_VERSION:
        require_schema(document)
        return list(document["frames"]), {"input_format": "golden", "metadata": document}

    if isinstance(document, dict) and document.get("format") == CODYCO_LOG_FORMAT_V1:
        return _load_codyco_joint_log_v1(document), {
            "input_format": CODYCO_LOG_FORMAT_V1,
            "metadata": document,
        }

    if isinstance(document, dict) and "frames" in document:
        frames = document["frames"]
        _validate_raw_frames(frames)
        return frames, {"input_format": "frames-wrapper", "metadata": document}

    if isinstance(document, list):
        _validate_raw_frames(document)
        return document, {"input_format": "frame-list"}

    raise ValueError(f"unsupported log format: {path}")


def _load_jsonl(text: str) -> list[dict[str, object]]:
    frames: list[dict[str, object]] = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        payload = json.loads(line)
        frames.append(_normalize_frame(payload, line_number))
    _validate_raw_frames(frames)
    return frames


def _load_codyco_joint_log_v1(document: dict[str, object]) -> list[dict[str, object]]:
    joint_order = [str(name) for name in document["joint_order"]]
    frames: list[dict[str, object]] = []
    for index, sample in enumerate(document["samples"]):
        positions = [float(value) for value in sample["positions"]]
        if len(positions) != len(joint_order):
            raise ValueError(
                f"sample {index} has {len(positions)} positions but joint_order has "
                f"{len(joint_order)} names"
            )
        joints = dict(zip(joint_order, positions, strict=True))
        frames.append({"t": float(sample["t"]), "joints": joints})
    _validate_raw_frames(frames)
    return frames


def _normalize_frame(payload: dict[str, object], line_number: int) -> dict[str, object]:
    if "t" not in payload:
        raise ValueError(f"line {line_number}: frame is missing field 't'")
    if "joints" in payload:
        joints = {str(k): float(v) for k, v in payload["joints"].items()}  # type: ignore[arg-type]
    elif "positions" in payload and "joint_order" in payload:
        joint_order = [str(name) for name in payload["joint_order"]]  # type: ignore[index]
        positions = [float(value) for value in payload["positions"]]  # type: ignore[arg-type]
        joints = dict(zip(joint_order, positions, strict=True))
    else:
        raise ValueError(f"line {line_number}: frame must contain joints or positions")
    return {"t": float(payload["t"]), "joints": joints}


def _validate_raw_frames(frames: list[dict[str, object]]) -> None:
    if not frames:
        raise ValueError("log contains no frames")
    previous_t = -1.0
    for index, frame in enumerate(frames):
        if "t" not in frame or "joints" not in frame:
            raise ValueError(f"frame {index} must contain t and joints")
        t_value = float(frame["t"])
        if t_value < previous_t:
            raise ValueError(f"frame times must be non-decreasing (frame {index})")
        previous_t = t_value


def load_csv_frames(path: Path) -> list[dict[str, object]]:
    frames: list[dict[str, object]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or "time" not in reader.fieldnames:
            raise ValueError("CSV log must include a 'time' column")
        joint_columns = [name for name in reader.fieldnames if name != "time"]
        for row in reader:
            joints = {name: float(row[name]) for name in joint_columns}
            frames.append({"t": float(row["time"]), "joints": joints})
    _validate_raw_frames(frames)
    return frames


def load_raw_frames_auto(path: Path) -> tuple[list[dict[str, object]], dict[str, object]]:
    if path.suffix.lower() == ".csv":
        return load_csv_frames(path), {"input_format": "csv"}
    return load_raw_frames(path)


def select_arm_joints(frame: dict[str, object]) -> dict[str, float]:
    joints = frame["joints"]
    missing = [joint for joint in ARM_JOINTS if joint not in joints]
    if missing:
        raise ValueError(f"frame at t={frame['t']} is missing arm joints: {missing}")
    return {joint: float(joints[joint]) for joint in ARM_JOINTS}


def resample_frames(
    frames: list[dict[str, object]],
    *,
    rate_hz: float,
    joint_names: tuple[str, ...] = ARM_JOINTS,
) -> list[dict[str, object]]:
    if not frames:
        raise ValueError("cannot resample an empty trajectory")

    dt = 1.0 / rate_hz
    end_time = float(frames[-1]["t"])
    output: list[dict[str, object]] = []
    sample_t = float(frames[0]["t"])
    while sample_t <= end_time + 1e-9:
        joints = interpolate_frames(frames, sample_t, joint_names)
        output.append({"t": round(sample_t, 6), "joints": joints})
        sample_t += dt
    return output


def interpolate_frames(
    frames: list[dict[str, object]],
    time_seconds: float,
    joint_names: tuple[str, ...],
) -> dict[str, float]:
    if time_seconds <= float(frames[0]["t"]):
        return {name: float(frames[0]["joints"][name]) for name in joint_names}

    for previous, current in zip(frames, frames[1:]):
        t0 = float(previous["t"])
        t1 = float(current["t"])
        if t0 <= time_seconds <= t1:
            if t1 <= t0:
                return {name: float(current["joints"][name]) for name in joint_names}
            ratio = (time_seconds - t0) / (t1 - t0)
            return {
                name: lerp(float(previous["joints"][name]), float(current["joints"][name]), ratio)
                for name in joint_names
            }

    return {name: float(frames[-1]["joints"][name]) for name in joint_names}


def build_golden_from_codyco_log(
    path: Path,
    *,
    rate_hz: float = DEFAULT_RATE_HZ,
    source_detail: str | None = None,
    notes: str | None = None,
) -> dict[str, object]:
    raw_frames, import_info = load_raw_frames_auto(path)
    arm_frames = [{"t": frame["t"], "joints": select_arm_joints(frame)} for frame in raw_frames]
    frames = resample_frames(arm_frames, rate_hz=rate_hz)

    metadata = import_info.get("metadata", {})
    if isinstance(metadata, dict):
        sequence = str(metadata.get("sequence", "stage_test_tasks"))
        container = metadata.get("container_image")
        recorded_at = metadata.get("recorded_at")
    else:
        sequence = "stage_test_tasks"
        container = None
        recorded_at = None

    detail_parts = [f"Imported from {path.name} ({import_info['input_format']})"]
    if source_detail:
        detail_parts.append(source_detail)
    if container:
        detail_parts.append(f"container={container}")
    if recorded_at:
        detail_parts.append(f"recorded_at={recorded_at}")

    return {
        "schema_version": SCHEMA_VERSION,
        "name": sequence,
        "source": "codyco-log",
        "source_detail": "; ".join(detail_parts),
        "topic_prefix": TOPIC_PREFIX,
        "joints": list(ARM_JOINTS),
        "rate_hz": rate_hz,
        "duration_seconds": frames[-1]["t"] if frames else 0.0,
        "notes": notes or "",
        "import": {
            "path": str(path),
            "input_format": import_info["input_format"],
            "raw_frames": len(raw_frames),
            "resampled_frames": len(frames),
        },
        "frames": frames,
    }
