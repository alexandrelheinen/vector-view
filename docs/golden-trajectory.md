# Golden trajectory replay

The release demo no longer depends on ad-hoc shell joint tweens at capture
time. Motion is driven by a versioned **golden trajectory** file and replayed
deterministically in Gazebo Harmonic.

## File format

Golden trajectories live under `assets/trajectories/` as JSON documents:

| Field | Meaning |
| --- | --- |
| `schema_version` | Format version (currently `1`) |
| `name` | Trajectory identifier (`stage_test_tasks`) |
| `source` | Provenance (`animate-grasp-keyframes`, `codyco-log`, …) |
| `source_detail` | Human-readable recording notes |
| `topic_prefix` | Joint command namespace (`/grasp_demo`) |
| `joints` | Ordered arm joint names |
| `rate_hz` | Sample rate used when the file was recorded |
| `frames[]` | `{ "t": seconds, "joints": { "<name>": radians } }` |

Each frame publishes position commands to
`/grasp_demo/<joint_name>`, which the `JointPositionController` plugins in
`icub_contact_release` consume.

## Bootstrap vs canonical oracle

The committed file
`assets/trajectories/stage_test_tasks.golden.json` is a **bootstrap oracle**:
it is generated from the same keyframes as the former `animate_grasp.sh`
script. That gives deterministic, reviewable motion without maintaining CoDyCo
on Ubuntu 24.04.

When a canonical CoDyCo `StageTestTasks` recording is available from the legacy
stack, replace the bootstrap file:

```bash
# future: import a CoDyCo joint-command log
python3 scripts/record_golden_trajectory.py \
  --source codyco-log \
  --codyco-log /path/to/stage_test_tasks.log \
  --output assets/trajectories/stage_test_tasks.golden.json
```

Until the `codyco-log` importer exists, regenerate the bootstrap file with:

```bash
python3 scripts/record_golden_trajectory.py
```

## Replay

```bash
python3 scripts/replay_golden_trajectory.py \
  --trajectory assets/trajectories/stage_test_tasks.golden.json
```

`scripts/animate_grasp.sh` is now a thin wrapper around this replay command.

## Capture and verification

`scripts/capture_comparison.sh` replays the golden trajectory, captures camera
and contact evidence at a fixed hold-phase time, and checks motion parity in
`scripts/verify_no_regression.py`:

| ID | Requirement |
| --- | --- |
| M1 | Capture uses the committed golden trajectory (SHA-256 recorded in report) |
| M2 | Motion comes from `replay_golden_trajectory.py`, not inline shell tweens |
| M3 | Joint commands at capture time match the golden frame within tolerance |

## Recording procedure (CoDyCo legacy environment)

1. Run Gazebo Classic + `ISIRWholeBodyController --sequence StageTestTasks` in
   the pinned `codyco-superbuild` container.
2. Log every joint position command sent to the iCub control board at ≥20 Hz.
3. Export the log in the JSON frame format above with `source: codyco-log`.
4. Commit the replacement golden file and update `source_detail` with the
   container image digest and recording date.
5. Re-run `scripts/capture_comparison.sh`; the visual/contact requirements must
   still pass with the new oracle.
