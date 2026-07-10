#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "$ROOT/.env" ]; then
  # shellcheck source=/dev/null
  source "$ROOT/.env"
fi

export VECTOR_VIEW="${VECTOR_VIEW:-$ROOT}"
export PATH="$VECTOR_VIEW/build:${PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$VECTOR_VIEW/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$VECTOR_VIEW/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_USER_PATH="$VECTOR_VIEW/worlds:${GZ_SIM_USER_PATH:-}"

detect_terminal() {
  for t in gnome-terminal xfce4-terminal konsole xterm; do
    command -v "$t" &>/dev/null && echo "$t" && return
  done
  echo "xterm"
}

TERMINAL="$(detect_terminal)"

if [ ! -x "$VECTOR_VIEW/build/vector-gui" ]; then
  echo "warning: $VECTOR_VIEW/build/vector-gui not found." >&2
  echo "         Build the project first or run with tests-only if Gazebo/Qt6 are unavailable." >&2
fi

echo "1. Starting YARP Server to iCub control."
$TERMINAL --tab -e "yarpserver --write"
sleep 1

echo "2. Running robot world at Gazebo Sim (Harmonic)."
$TERMINAL --tab -e "gz sim -r $VECTOR_VIEW/worlds/robot.world"
sleep 6

echo "3. Opening the GUI interface for force analysis."
echo "3.1. topic path: /vectorview/iCub_fixed/r_hand"
$TERMINAL --tab -e "$VECTOR_VIEW/build/vectorGUI /vectorview/iCub_fixed/r_hand"
sleep 0.5

echo "3.2. topic path: /vectorview/iCub_fixed/l_hand"
$TERMINAL --tab -e "$VECTOR_VIEW/build/vectorGUI l_hand"
sleep 1

if [ -z "${CODYCO_SUPERBUILD_ROOT:-}" ]; then
  echo "4. Skipping ISIRWholeBodyController because CODYCO_SUPERBUILD_ROOT is unset."
  echo "   Set it in .env to launch the StageTestTasks demo automatically."
else
  echo "4. Running StageTestTasks sequence of ISIR Controller."
  $TERMINAL --tab -e "$CODYCO_SUPERBUILD_ROOT/build/install/bin/ISIRWholeBodyController --sequence StageTestTasks"
fi

echo " -------------------------------------------------------"
echo " Demo stack note: pair this world with gz-sim-yarp-plugins"
echo " (not gazebo-yarp-plugins) on Ubuntu 24.04 Noble."
echo " -------------------------------------------------------"
