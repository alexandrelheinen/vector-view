#!/usr/bin/env bash
set -euo pipefail

WORLD_NAME="${WORLD_NAME:-release_demo}"
MODEL_NAME="${MODEL_NAME:-hand_probe}"
POSE_X="${POSE_X:-0.365}"
POSE_Y="${POSE_Y:-0.0}"

set_pose() {
  local z="$1"
  gz service -s "/world/${WORLD_NAME}/set_pose" \
    --reqtype gz.msgs.Pose \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "name: \"${MODEL_NAME}\", position: {x: ${POSE_X}, y: ${POSE_Y}, z: ${z}}" \
    >/dev/null
}

echo "Animating ${MODEL_NAME}: approach, press, release..."

# Hover above the box
set_pose 0.85
sleep 0.8

# Press down onto the box top (z ≈ 0.675 for pad center)
for z in $(seq 0.85 -0.01 0.675); do
  set_pose "${z}"
  sleep 0.04
done

# Hold contact so force arrows are visible
sleep 2.0

# Release / retract
for z in $(seq 0.675 0.01 0.90); do
  set_pose "${z}"
  sleep 0.04
done

sleep 0.8
echo "Animation complete."
