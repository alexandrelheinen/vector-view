#!/usr/bin/env bash
# Capture side-by-side comparison during the grasp hold phase (contact + arrows).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/docs/images/execution_comparison.png}"
LEGACY="$ROOT/docs/images/execution_example.png"
WORKDIR="$(mktemp -d)"
CAPTURE_AT_SEC=9.5
PLOT_START_SEC=8.0

export VECTOR_VIEW="$ROOT"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/assets/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_USER_PATH="$ROOT/assets/worlds:${GZ_SIM_USER_PATH:-}"

cleanup() {
  killall -9 gz 2>/dev/null || true
}
trap cleanup EXIT

cleanup
sleep 2

DISPLAY= gz sim -s -r --headless-rendering "$ROOT/assets/worlds/release_demo.world" >/tmp/gz_capture.log 2>&1 &
sleep 16

"$ROOT/scripts/animate_grasp.sh" >"$WORKDIR/animate.log" 2>&1 &
animate_pid=$!

sleep "$PLOT_START_SEC"
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/r_hand --label r_hand_contact \
  --output "$WORKDIR/r_hand_plot.png" --seconds 3 &
plot_r=$!
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/l_hand --label l_hand_contact \
  --output "$WORKDIR/l_hand_plot.png" --seconds 3 &
plot_l=$!

sleep "$(awk -v cap="$CAPTURE_AT_SEC" -v plot="$PLOT_START_SEC" 'BEGIN { printf "%.2f", cap - plot }')"

python3 "$ROOT/scripts/save_camera_frame.py" --output "$WORKDIR/gazebo_raw.png" &
frame_pid=$!
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/r_hand --label r_hand_contact \
  --output "$WORKDIR/r_hand_plot.png" --seconds 3 &
plot_r=$!
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/l_hand --label l_hand_contact \
  --output "$WORKDIR/l_hand_plot.png" --seconds 3 &
plot_l=$!
wait "$frame_pid"
wait "$plot_r" || true
wait "$plot_l" || true
python3 "$ROOT/scripts/overlay_force_arrows.py" \
  --image "$WORKDIR/gazebo_raw.png" --output "$WORKDIR/gazebo.png"
wait "$animate_pid" || true

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
