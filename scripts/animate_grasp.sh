#!/usr/bin/env bash
# Animate the iCub arms to press and release the box, reproducing the 2015
# internship grasp demo without the external CoDyCo controller stack.
set -euo pipefail

publish_joint() {
  local topic="$1"
  local value="$2"
  gz topic -t "$topic" -m gz.msgs.Double -p "data: ${value}" >/dev/null
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

echo "Grasp demo: idle pose"
animate_pose 1.5 \
  "/grasp_demo/l_shoulder_pitch,0.0,0.0" \
  "/grasp_demo/l_shoulder_roll,0.3,0.3" \
  "/grasp_demo/l_elbow,0.4,0.4" \
  "/grasp_demo/l_wrist_pitch,0.0,0.0" \
  "/grasp_demo/r_shoulder_pitch,0.0,0.0" \
  "/grasp_demo/r_shoulder_roll,0.3,0.3" \
  "/grasp_demo/r_elbow,0.4,0.4" \
  "/grasp_demo/r_wrist_pitch,0.0,0.0"

echo "Grasp demo: reach toward the box"
animate_pose 3.0 \
  "/grasp_demo/l_shoulder_pitch,0.0,-0.95" \
  "/grasp_demo/l_shoulder_roll,0.3,1.35" \
  "/grasp_demo/l_elbow,0.4,1.35" \
  "/grasp_demo/l_wrist_pitch,0.0,-0.55" \
  "/grasp_demo/r_shoulder_pitch,0.0,-0.95" \
  "/grasp_demo/r_shoulder_roll,0.3,1.35" \
  "/grasp_demo/r_elbow,0.4,1.35" \
  "/grasp_demo/r_wrist_pitch,0.0,-0.55"

echo "Grasp demo: press the box (contact arrows appear)"
animate_pose 2.5 \
  "/grasp_demo/l_shoulder_pitch,-0.95,-1.25" \
  "/grasp_demo/l_shoulder_roll,1.35,1.45" \
  "/grasp_demo/l_elbow,1.35,0.55" \
  "/grasp_demo/l_wrist_pitch,-0.55,-0.35" \
  "/grasp_demo/r_shoulder_pitch,-0.95,-1.25" \
  "/grasp_demo/r_shoulder_roll,1.35,1.45" \
  "/grasp_demo/r_elbow,1.35,0.55" \
  "/grasp_demo/r_wrist_pitch,-0.55,-0.35"

echo "Grasp demo: hold contact"
sleep 2.0

echo "Grasp demo: release and retract"
animate_pose 3.0 \
  "/grasp_demo/l_shoulder_pitch,-1.25,0.0" \
  "/grasp_demo/l_shoulder_roll,1.45,0.3" \
  "/grasp_demo/l_elbow,0.55,0.4" \
  "/grasp_demo/l_wrist_pitch,-0.35,0.0" \
  "/grasp_demo/r_shoulder_pitch,-1.25,0.0" \
  "/grasp_demo/r_shoulder_roll,1.45,0.3" \
  "/grasp_demo/r_elbow,0.55,0.4" \
  "/grasp_demo/r_wrist_pitch,-0.35,0.0"

sleep 1.0
echo "Grasp demo animation complete"
