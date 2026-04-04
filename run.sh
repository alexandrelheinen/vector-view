#!/usr/bin/env bash

# Detect an available terminal emulator.
detect_terminal() {
  for t in gnome-terminal xfce4-terminal konsole xterm; do
    command -v "$t" &>/dev/null && echo "$t" && return
  done
  echo "xterm"
}
TERMINAL=$(detect_terminal)

# Helper: open a new terminal tab/window running CMD.
run_in_terminal() {
  local cmd="$1"
  case "$TERMINAL" in
    gnome-terminal) gnome-terminal --tab -- bash -c "$cmd; exec bash" ;;
    xfce4-terminal) xfce4-terminal --tab --command="bash -c '$cmd; exec bash'" ;;
    konsole)        konsole --new-tab -e bash -c "$cmd; exec bash" ;;
    *)              xterm -e bash -c "$cmd; exec bash" & ;;
  esac
}

echo "1. Starting YARP Server to iCub control."
run_in_terminal "yarpserver --write"
sleep 1

echo "2. Running [robot.world] at Gazebo simulator."
run_in_terminal "gazebo robot.world"
sleep 6

echo "3. Opening the GUI Interface for force analysis."
echo "3.1. topic path: /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact"
run_in_terminal "./build/vectorGUI /gazebo/default/iCub_fixed/iCub/r_hand/r_hand_contact"
sleep 0.5

echo "3.2. topic path: /gazebo/default/iCub_fixed/iCub/l_hand/l_hand_contact"
run_in_terminal "./build/vectorGUI l_hand"  # short link-name form
sleep 1

echo "4. Running [StageTestTasks] sequence of ISIR Controller."
run_in_terminal "$CODYCO_SUPERBUILD_ROOT/build/install/bin/ISIRWholeBodyController --sequence StageTestTasks"
echo " -------------------------------------------------------"
