#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "$ROOT/.env" ]; then
  # shellcheck source=/dev/null
  source "$ROOT/.env"
fi

export VECTOR_VIEW="${VECTOR_VIEW:-$ROOT}"
export PATH="$VECTOR_VIEW/build:${PATH}"
export GAZEBO_PLUGIN_PATH="$VECTOR_VIEW/build:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="$VECTOR_VIEW/models:${GAZEBO_MODEL_PATH:-}"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "error: gnome-terminal is required by this demo script." >&2
  exit 1
fi

if [ ! -x "$VECTOR_VIEW/build/vectorGUI" ]; then
  echo "warning: $VECTOR_VIEW/build/vectorGUI not found." >&2
  echo "         Build the project first or run with tests-only if Gazebo/Qt4 are unavailable." >&2
fi

echo "1. Starting YARP Server to iCub control."
gnome-terminal --tab -e "yarpserver --write"
sleep 1

echo "2. Running robot world at Gazebo simulator."
gnome-terminal --tab -e "gazebo $VECTOR_VIEW/worlds/robot.world"
sleep 6

echo "3. Opening the GUI interface for force analysis."
echo "3.1. topic path: /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact"
gnome-terminal --tab -e "$VECTOR_VIEW/build/vectorGUI /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact"
sleep 0.5

echo "3.2. topic path: /gazebo/default/iCub_fixed/iCub/l_hand/l_hand_contact"
gnome-terminal --tab -e "$VECTOR_VIEW/build/vectorGUI l_hand"
sleep 1

if [ -z "${CODYCO_SUPERBUILD_ROOT:-}" ]; then
  echo "4. Skipping ISIRWholeBodyController because CODYCO_SUPERBUILD_ROOT is unset."
  echo "   Set it in .env to launch the StageTestTasks demo automatically."
else
  echo "4. Running StageTestTasks sequence of ISIR Controller."
  gnome-terminal --tab -e "$CODYCO_SUPERBUILD_ROOT/build/install/bin/ISIRWholeBodyController --sequence StageTestTasks"
fi

echo " -------------------------------------------------------"
