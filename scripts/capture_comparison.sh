#!/usr/bin/env bash
# Capture an honest side-by-side comparison during the grasp hold phase.
# Uses the headless release camera and real VectorView geometry (blue arrows on hands).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/docs/images/execution_comparison.png}"
LEGACY="$ROOT/docs/images/execution_example.png"
WORKDIR="$(mktemp -d)"
SIM_WARMUP_SEC=12
CAPTURE_AT_SEC=10
PLOT_START_SEC=8

export VECTOR_VIEW="$ROOT"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/assets/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_USER_PATH="$ROOT/assets/worlds:${GZ_SIM_USER_PATH:-}"

cleanup() {
  killall -9 gz 2>/dev/null || true
}
trap cleanup EXIT

killall -9 gz 2>/dev/null || true
sleep 1

DISPLAY= gz sim -s -r --headless-rendering "$ROOT/assets/worlds/release_demo.world" >/tmp/gz_capture.log 2>&1 &
SIM_PID=$!
sleep "$SIM_WARMUP_SEC"

"$ROOT/scripts/animate_grasp.sh" >/tmp/capture_anim.log 2>&1 &
ANIM_PID=$!

sleep "$PLOT_START_SEC"
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/r_hand --label r_hand_contact \
  --output "$WORKDIR/r_hand_plot.png" --seconds 4 &
PLOT_R=$!
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/l_hand --label l_hand_contact \
  --output "$WORKDIR/l_hand_plot.png" --seconds 4 &
PLOT_L=$!

sleep "$(awk -v c="$CAPTURE_AT_SEC" -v p="$PLOT_START_SEC" 'BEGIN { printf "%.2f", c - p }')"
python3 "$ROOT/scripts/save_camera_frame.py" --output "$WORKDIR/gazebo.png" --timeout 15

wait "$PLOT_R" "$PLOT_L" "$ANIM_PID" 2>/dev/null || true
kill "$SIM_PID" 2>/dev/null || true
killall -9 gz 2>/dev/null || true

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
