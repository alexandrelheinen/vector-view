# CoDyCo joint-command log format

Golden trajectories in this repository can be produced from three interchange
formats. All of them describe **irregular-time joint position commands** in
radians. The importer resamples to the golden replay rate (default 20 Hz) and
keeps only the 14 arm joints used by the release demo.

## Supported inputs

| Extension | Format | Typical origin |
| --- | --- | --- |
| `.json` | `vectorview-codyco-joint-log-v1` | `scripts/legacy/record_stage_test_tasks_yarp.py` |
| `.json` | `{ "frames": [ { "t", "joints" } ] }` wrapper | manual export |
| `.jsonl` | one `{ "t", "joints" }` object per line | streaming logger |
| `.csv` | `time` column + one column per joint | spreadsheet / MATLAB export |
| `.json` | golden `schema_version: 1` | re-normalize an existing golden file |

Import command:

```bash
python3 scripts/record_golden_trajectory.py \
  --source codyco-log \
  --codyco-log /path/to/log.json \
  --output assets/trajectories/stage_test_tasks.golden.json
```

## `vectorview-codyco-joint-log-v1`

```json
{
  "format": "vectorview-codyco-joint-log-v1",
  "sequence": "StageTestTasks",
  "rate_hz": 50.0,
  "recorded_at": "2015-08-01T12:00:00+00:00",
  "container_image": "codyco-superbuild:2015-pin",
  "robot_prefix": "icubGazeboSim",
  "ports": {
    "left_arm": "/icubGazeboSim/left_arm",
    "right_arm": "/icubGazeboSim/right_arm"
  },
  "joint_order": [
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
    "r_wrist_yaw"
  ],
  "samples": [
    { "t": 0.0, "positions": [ -0.52, 0.52, 0.0, 0.785, 0.0, 0.0, 0.698, -0.52, 0.52, 0.0, 0.785, 0.0, 0.0, 0.698 ] }
  ]
}
```

Rules:

- `joint_order` and each `positions` array must have the same length.
- `t` values must be non-decreasing seconds from trajectory start.
- All 14 arm joints above must be present (either in `joint_order` or in per-frame `joints` maps).

## JSONL

```json
{"t": 0.0, "joints": {"l_shoulder_pitch": -0.52, "l_shoulder_roll": 0.52}}
{"t": 0.05, "joints": {"l_shoulder_pitch": -0.53, "l_shoulder_roll": 0.53}}
```

Lines starting with `#` are ignored.

## CSV

```csv
time,l_shoulder_pitch,l_shoulder_roll,l_shoulder_yaw,l_elbow,l_wrist_prosup,l_wrist_pitch,l_wrist_yaw,r_shoulder_pitch,r_shoulder_roll,r_shoulder_yaw,r_elbow,r_wrist_prosup,r_wrist_pitch,r_wrist_yaw
0.0,-0.52,0.52,0.0,0.785,0.0,0.0,0.698,-0.52,0.52,0.0,0.785,0.0,0.0,0.698
```

## Resampling

The importer linearly interpolates each joint between irregular samples, then
emits evenly spaced golden frames at `--rate-hz` (default 20). This matches the
replay clock used by `scripts/replay_golden_trajectory.py`.

## Fixtures

Compact examples for unit tests live under `tests/fixtures/`:

- `stage_test_tasks.codyco-log.json`
- `stage_test_tasks.frames.jsonl`
- `stage_test_tasks.frames.csv`
