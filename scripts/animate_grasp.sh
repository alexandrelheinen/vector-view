#!/usr/bin/env bash
# Animate the iCub arms to press and release the box by replaying the golden
# StageTestTasks joint trajectory. Legs, torso, and head remain under the
# JointPositionController plugins in icub_contact_release.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRAJECTORY="${GOLDEN_TRAJECTORY:-$ROOT/assets/trajectories/stage_test_tasks.golden.json}"

exec python3 "$ROOT/scripts/replay_golden_trajectory.py" --trajectory "$TRAJECTORY" "$@"
