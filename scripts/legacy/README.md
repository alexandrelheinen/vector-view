# Legacy CoDyCo trajectory recording

Use this workflow inside the pinned **CoDyCo / Gazebo Classic** container once
`ISIRWholeBodyController --sequence StageTestTasks` is running.

## 1. Start the legacy demo stack

```bash
yarpserver --write
# Gazebo Classic with the 2015 iCub + box world
ISIRWholeBodyController --sequence StageTestTasks
```

Confirm the robot prefix matches the YARP control ports. The iCub Gazebo plugins
in this repository default to `icubGazeboSim`, which yields:

- `/icubGazeboSim/left_arm`
- `/icubGazeboSim/right_arm`

## 2. Record arm joint commands

```bash
python3 scripts/legacy/record_stage_test_tasks_yarp.py \
  --robot-prefix icubGazeboSim \
  --duration 20 \
  --rate-hz 50 \
  --container-image "<image digest or tag>" \
  --output /tmp/stage_test_tasks.codyco-log.json
```

The script requires YARP Python bindings (`import yarp`). It writes a
`vectorview-codyco-joint-log-v1` document (see `docs/codyco-log-format.md`).

## 3. Import into the Harmonic golden file

Copy the log out of the container, then on Ubuntu 24.04:

```bash
python3 scripts/record_golden_trajectory.py \
  --source codyco-log \
  --codyco-log /tmp/stage_test_tasks.codyco-log.json \
  --rate-hz 20 \
  --source-detail "CoDyCo StageTestTasks canonical recording" \
  --output assets/trajectories/stage_test_tasks.golden.json
```

## 4. Verify on Noble

```bash
cmake --build build
scripts/capture_comparison.sh
```

The capture must still print `NO-REGRESSION PASS` with the imported oracle.

## Alternate export formats

If you already have a joint log in CSV or JSONL, import it directly. Required
columns/fields are documented in `docs/codyco-log-format.md`.
