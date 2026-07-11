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
PLOT_START_SEC=7
PLOT_SECONDS=5

export VECTOR_VIEW="$ROOT"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/assets/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_USER_PATH="$ROOT/assets/worlds:${GZ_SIM_USER_PATH:-}"

cleanup_gz() {
  pkill -9 -x gz 2>/dev/null || true
}
trap cleanup_gz EXIT

cleanup_gz
sleep 1

DISPLAY= gz sim -s -r --headless-rendering "$ROOT/assets/worlds/release_demo.world" >/tmp/gz_capture.log 2>&1 &
SIM_PID=$!
sleep "$SIM_WARMUP_SEC"

"$ROOT/scripts/animate_grasp.sh" >/tmp/capture_anim.log 2>&1 &
ANIM_PID=$!

sleep "$PLOT_START_SEC"
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/r_hand --label r_hand_contact \
  --output "$WORKDIR/r_hand_plot.png" --seconds "$PLOT_SECONDS" &
PLOT_R=$!
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/l_hand --label l_hand_contact \
  --output "$WORKDIR/l_hand_plot.png" --seconds "$PLOT_SECONDS" &
PLOT_L=$!

sleep "$(awk -v c="$CAPTURE_AT_SEC" -v p="$PLOT_START_SEC" 'BEGIN { printf "%.2f", c - p }')"
python3 "$ROOT/scripts/save_camera_frame.py" --output "$WORKDIR/gazebo.png" --timeout 15

wait "$PLOT_R" "$PLOT_L" "$ANIM_PID" 2>/dev/null || true
kill "$SIM_PID" 2>/dev/null || true
cleanup_gz

# Match the 2015 layout: plots top-left over the 3D view.
convert "$WORKDIR/r_hand_plot.png" "$WORKDIR/l_hand_plot.png" +append "$WORKDIR/plots_row.png"
convert "$WORKDIR/gazebo.png" -resize 1280x720 -background '#bfbfbf' -gravity center \
  -extent 1280x720 "$WORKDIR/gazebo_view.png"
convert "$WORKDIR/plots_row.png" -resize 1016x312 "$WORKDIR/plots_row.png"
convert "$WORKDIR/gazebo_view.png" "$WORKDIR/plots_row.png" \
  -gravity northwest -geometry +8+48 -composite "$WORKDIR/current_view.png"
convert "$WORKDIR/current_view.png" -gravity north -background white -splice 0x40 \
  -fill black -pointsize 28 -annotate +20+8 "v2.0+ (Gazebo Harmonic)" "$WORKDIR/current.png"

convert "$LEGACY" -resize 1280x720 -gravity north -background white -splice 0x40 \
  -fill black -pointsize 28 -annotate +20+8 "2015 reference (Gazebo Classic)" "$WORKDIR/legacy.png"
convert "$WORKDIR/legacy.png" "$WORKDIR/current.png" +append -background white \
  -gravity center -append -bordercolor '#333333' -border 8 "$OUTPUT"

echo "Comparison saved to $OUTPUT"
ls -lh "$OUTPUT"
python3 - <<PY
import json
from pathlib import Path
for hand in ("r_hand", "l_hand"):
    meta = Path("$WORKDIR") / f"{hand}_plot.json"
    if meta.exists():
        print(hand, meta.read_text())
PY
