#!/usr/bin/env bash
# Animate the iCub arms to press and release the box, reproducing the 2015
# internship grasp demo without the external CoDyCo controller stack.
#
# Legs, torso, and head are held by JointPositionController plugins in
# icub_contact_release (same PIDs as the original gazebo_yarp_controlboard).
set -euo pipefail

publish_joint() {
  local topic="$1"
  local value="$2"
  timeout 2 gz topic -t "$topic" -m gz.msgs.Double -p "data: ${value}" >/dev/null 2>&1 || true
}

lerp() {
  local start="$1"
  local end="$2"
  local t="$3"
  awk -v s="$start" -v e="$end" -v t="$t" 'BEGIN { printf "%.6f", s + (e - s) * t }'
}

animate_pair() {
  local topic="$1"
  local start="$2"
  local end="$3"
  local duration="$4"
  local steps="${5:-20}"

  local step_sleep
  step_sleep="$(awk -v d="$duration" -v n="$steps" 'BEGIN { printf "%.4f", d / n }')"

  local i t value
  for ((i = 0; i <= steps; ++i)); do
    t="$(awk -v i="$i" -v n="$steps" 'BEGIN { printf "%.6f", i / n }')"
    value="$(lerp "$start" "$end" "$t")"
    publish_joint "$topic" "$value"
    sleep "$step_sleep"
  done
}

animate_pose() {
  local duration="$1"
  shift
  local -a specs=("$@")
  local spec topic start end

  for spec in "${specs[@]}"; do
    IFS=',' read -r topic start end <<<"$spec"
    animate_pair "$topic" "$start" "$end" "$duration" &
  done
  wait
}

hold_pose() {
  local duration="$1"
  local interval="${2:-0.1}"
  shift 2
  local -a specs=("$@")
  local spec topic value
  local elapsed=0

  while awk -v e="$elapsed" -v d="$duration" 'BEGIN { exit !(e < d) }'; do
    for spec in "${specs[@]}"; do
      IFS=',' read -r topic value <<<"$spec"
      publish_joint "$topic" "$value"
    done
    sleep "$interval"
    elapsed="$(awk -v e="$elapsed" -v i="$interval" 'BEGIN { printf "%.4f", e + i }')"
  done
}

# 2015 default arm configuration from icub.sdf initialConfiguration
SP=-0.52
SR=0.52
SY=0
EL=0.785
WP=0
WY=0
WYAW=0.698

# Reach and press targets from the original CoDyCo StageTestTasks grasp sequence.
REACH_SP=-0.95
REACH_SR=1.35
REACH_EL=1.35
REACH_WY=-0.55

PRESS_SP=-1.25
PRESS_SR=1.45
PRESS_EL=0.55
PRESS_WY=-0.35

echo "Grasp demo: hold standing pose"
sleep 1.0

echo "Grasp demo: reach toward the box"
animate_pose 3.0 \
  "/grasp_demo/l_shoulder_pitch,${SP},${REACH_SP}" \
  "/grasp_demo/l_shoulder_roll,${SR},${REACH_SR}" \
  "/grasp_demo/l_shoulder_yaw,${SY},${SY}" \
  "/grasp_demo/l_elbow,${EL},${REACH_EL}" \
  "/grasp_demo/l_wrist_prosup,${WP},${WP}" \
  "/grasp_demo/l_wrist_pitch,${WY},${REACH_WY}" \
  "/grasp_demo/l_wrist_yaw,${WYAW},${WYAW}" \
  "/grasp_demo/r_shoulder_pitch,${SP},${REACH_SP}" \
  "/grasp_demo/r_shoulder_roll,${SR},${REACH_SR}" \
  "/grasp_demo/r_shoulder_yaw,${SY},${SY}" \
  "/grasp_demo/r_elbow,${EL},${REACH_EL}" \
  "/grasp_demo/r_wrist_prosup,${WP},${WP}" \
  "/grasp_demo/r_wrist_pitch,${WY},${REACH_WY}" \
  "/grasp_demo/r_wrist_yaw,${WYAW},${WYAW}"

echo "Grasp demo: press the box (contact arrows appear)"
animate_pose 2.5 \
  "/grasp_demo/l_shoulder_pitch,${REACH_SP},${PRESS_SP}" \
  "/grasp_demo/l_shoulder_roll,${REACH_SR},${PRESS_SR}" \
  "/grasp_demo/l_shoulder_yaw,${SY},0.12" \
  "/grasp_demo/l_elbow,${REACH_EL},${PRESS_EL}" \
  "/grasp_demo/l_wrist_pitch,${REACH_WY},${PRESS_WY}" \
  "/grasp_demo/r_shoulder_pitch,${REACH_SP},${PRESS_SP}" \
  "/grasp_demo/r_shoulder_roll,${REACH_SR},${PRESS_SR}" \
  "/grasp_demo/r_shoulder_yaw,${SY},-0.12" \
  "/grasp_demo/r_elbow,${REACH_EL},${PRESS_EL}" \
  "/grasp_demo/r_wrist_pitch,${REACH_WY},${PRESS_WY}"

echo "Grasp demo: hold contact (force arrows visible)"
hold_pose 5.0 \
  "/grasp_demo/l_shoulder_pitch,${PRESS_SP}" \
  "/grasp_demo/l_shoulder_roll,${PRESS_SR}" \
  "/grasp_demo/l_shoulder_yaw,0.12" \
  "/grasp_demo/l_elbow,${PRESS_EL}" \
  "/grasp_demo/l_wrist_pitch,${PRESS_WY}" \
  "/grasp_demo/r_shoulder_pitch,${PRESS_SP}" \
  "/grasp_demo/r_shoulder_roll,${PRESS_SR}" \
  "/grasp_demo/r_shoulder_yaw,-0.12" \
  "/grasp_demo/r_elbow,${PRESS_EL}" \
  "/grasp_demo/r_wrist_pitch,${PRESS_WY}"

echo "Grasp demo: release and return to standing"
animate_pose 3.0 \
  "/grasp_demo/l_shoulder_pitch,${PRESS_SP},${SP}" \
  "/grasp_demo/l_shoulder_roll,${PRESS_SR},${SR}" \
  "/grasp_demo/l_shoulder_yaw,${SY},${SY}" \
  "/grasp_demo/l_elbow,${PRESS_EL},${EL}" \
  "/grasp_demo/l_wrist_prosup,${WP},${WP}" \
  "/grasp_demo/l_wrist_pitch,${PRESS_WY},${WY}" \
  "/grasp_demo/l_wrist_yaw,${WYAW},${WYAW}" \
  "/grasp_demo/r_shoulder_pitch,${PRESS_SP},${SP}" \
  "/grasp_demo/r_shoulder_roll,${PRESS_SR},${SR}" \
  "/grasp_demo/r_shoulder_yaw,${SY},${SY}" \
  "/grasp_demo/r_elbow,${PRESS_EL},${EL}" \
  "/grasp_demo/r_wrist_prosup,${WP},${WP}" \
  "/grasp_demo/r_wrist_pitch,${PRESS_WY},${WY}" \
  "/grasp_demo/r_wrist_yaw,${WYAW},${WYAW}"

sleep 1.0
echo "Grasp demo animation complete"
