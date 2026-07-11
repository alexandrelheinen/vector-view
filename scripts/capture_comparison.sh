#!/usr/bin/env bash
# One-shot capture: press pose, record clip + plots, build comparison image.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/docs/images/execution_comparison.png}"
LEGACY="$ROOT/docs/images/execution_example.png"
WORKDIR="$(mktemp -d)"
RECORD_TOPIC="/camera/record_video"

export VECTOR_VIEW="$ROOT"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/assets/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_USER_PATH="$ROOT/assets/worlds:${GZ_SIM_USER_PATH:-}"

cleanup() {
  killall -9 gz 2>/dev/null || true
}
trap cleanup EXIT

killall -9 gz 2>/dev/null || true
sleep 2

DISPLAY= gz sim -s -r --headless-rendering "$ROOT/assets/worlds/release_demo.world" >/tmp/gz_capture.log 2>&1 &
sleep 14

publish_joint() {
  timeout 2 gz topic -t "$1" -m gz.msgs.Double -p "data: $2" >/dev/null 2>&1 || true
}

press_pose() {
  publish_joint /grasp_demo/l_shoulder_pitch -1.25
  publish_joint /grasp_demo/l_shoulder_roll 1.45
  publish_joint /grasp_demo/l_elbow 0.55
  publish_joint /grasp_demo/l_wrist_pitch -0.35
  publish_joint /grasp_demo/r_shoulder_pitch -1.25
  publish_joint /grasp_demo/r_shoulder_roll 1.45
  publish_joint /grasp_demo/r_elbow 0.55
  publish_joint /grasp_demo/r_wrist_pitch -0.35
}

# Reach, then press, then hold contact.
"$ROOT/scripts/animate_grasp.sh" >"$WORKDIR/animate.log" 2>&1 &
animate_pid=$!

# Capture during the scripted five-second hold window.
sleep 18
press_pose
sleep 1

SHORT_VIDEO="$WORKDIR/contact.mp4"
gz service -s "$RECORD_TOPIC" \
  --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 10000 \
  --req "start: true, format: 'mp4', save_filename: '${SHORT_VIDEO}'"

for _ in $(seq 1 20); do
  press_pose
  sleep 0.25
done

python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/r_hand --label r_hand_contact \
  --output "$WORKDIR/r_hand_plot.png" --seconds 4
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/l_hand --label l_hand_contact \
  --output "$WORKDIR/l_hand_plot.png" --seconds 4

gz service -s "$RECORD_TOPIC" \
  --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 30000 \
  --req "stop: true"
kill "$animate_pid" 2>/dev/null || true
sleep 2

ffmpeg -y -sseof -1 -i "$SHORT_VIDEO" -frames:v 1 "$WORKDIR/gazebo.png" >/dev/null 2>&1
convert "$WORKDIR/r_hand_plot.png" "$WORKDIR/l_hand_plot.png" +append "$WORKDIR/gui_row.png"
convert "$WORKDIR/gui_row.png" "$WORKDIR/gazebo.png" -gravity center -append \
  -bordercolor white -border 12 "$WORKDIR/current_stack.png"
convert "$LEGACY" -resize 1280x720 -gravity north -background white -splice 0x40 \
  -fill black -pointsize 28 -annotate +20+8 "2015 reference (Gazebo Classic)" "$WORKDIR/legacy.png"
convert "$WORKDIR/current_stack.png" -resize 1280x720 -gravity north -background white -splice 0x40 \
  -fill black -pointsize 28 -annotate +20+8 "v2.0+ (Gazebo Harmonic)" "$WORKDIR/current.png"
convert "$WORKDIR/legacy.png" "$WORKDIR/current.png" +append -background white \
  -gravity center -append -bordercolor '#333333' -border 8 "$OUTPUT"

echo "Comparison saved to $OUTPUT"
ls -lh "$OUTPUT"
