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
PLOT_SECONDS=12
RAW_PROOF="$ROOT/docs/images/execution_current_raw.png"
ARROW_PROOF="$ROOT/docs/images/execution_arrows_raw.png"
REPORT="$ROOT/docs/no-regression-report.json"

export VECTOR_VIEW="$ROOT"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/assets/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_USER_PATH="$ROOT/assets/worlds:${GZ_SIM_USER_PATH:-}"

cleanup_gz() {
  pkill -9 -f '^gz sim( |$)' 2>/dev/null || true
}
trap cleanup_gz EXIT

cleanup_gz
sleep 1

DISPLAY= gz sim -s -r --headless-rendering "$ROOT/assets/worlds/release_demo.world" >/tmp/gz_capture.log 2>&1 &
SIM_PID=$!
sleep "$SIM_WARMUP_SEC"

# Start recording before motion. The plots therefore show the zero-force
# baseline, contact onset, and loaded hold phase instead of a cropped decay.
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/r_hand --label r_hand_contact \
  --output "$WORKDIR/r_hand_plot.png" --seconds "$PLOT_SECONDS" &
PLOT_R=$!
python3 "$ROOT/scripts/record_force_plot.py" \
  --topic /vectorview/iCub_fixed/l_hand --label l_hand_contact \
  --output "$WORKDIR/l_hand_plot.png" --seconds "$PLOT_SECONDS" &
PLOT_L=$!

sleep 0.5
"$ROOT/scripts/animate_grasp.sh" >/tmp/capture_anim.log 2>&1 &
ANIM_PID=$!

sleep "$CAPTURE_AT_SEC"

# Record the contact messages independently of the rendered image. These files
# are consumed by verify_no_regression.py; capture fails if either hand is not
# touching object::main::collision.
timeout 5 gz topic -e -t /vectorview/iCub_fixed/r_hand -n 1 \
  >"$WORKDIR/r_hand_contact.txt" 2>&1 &
CONTACT_R=$!
timeout 5 gz topic -e -t /vectorview/iCub_fixed/l_hand -n 1 \
  >"$WORKDIR/l_hand_contact.txt" 2>&1 &
CONTACT_L=$!
python3 "$ROOT/scripts/save_camera_frame.py" \
  --topic /release_camera --output "$WORKDIR/gazebo_raw.png" --timeout 15 &
FRAME_SIDE=$!
python3 "$ROOT/scripts/save_camera_frame.py" \
  --topic /arrow_camera --output "$WORKDIR/arrows_raw.png" --timeout 15 &
FRAME_ARROWS=$!
wait "$FRAME_SIDE" "$FRAME_ARROWS"
wait "$CONTACT_R" "$CONTACT_L"

wait "$PLOT_R" "$PLOT_L"
kill "$ANIM_PID" 2>/dev/null || true
kill "$SIM_PID" 2>/dev/null || true
cleanup_gz

# This verifier runs on the unmodified /release_camera frame and raw topic
# evidence, before any layout work. It is the executable no-regression proof.
python3 "$ROOT/scripts/verify_no_regression.py" \
  --comparison-frame "$WORKDIR/gazebo_raw.png" \
  --arrow-frame "$WORKDIR/arrows_raw.png" \
  --left-contact "$WORKDIR/l_hand_contact.txt" \
  --right-contact "$WORKDIR/r_hand_contact.txt" \
  --left-plot "$WORKDIR/l_hand_plot.json" \
  --right-plot "$WORKDIR/r_hand_plot.json" \
  --output "$REPORT"

cp "$WORKDIR/gazebo_raw.png" "$RAW_PROOF"
cp "$WORKDIR/arrows_raw.png" "$ARROW_PROOF"

# Comparable layout with strict separation: the unobstructed camera view is
# above, and the two plots are below it. The side view matches the reference;
# the front-oblique view proves that two distinct arrows exist. No window
# covers either camera frame, and no simulation content is painted onto them.
convert "$WORKDIR/gazebo_raw.png" -crop 700x650+200+40 +repage \
  -resize 520x480 -background '#d0d0d0' -gravity center \
  -extent 640x500 "$WORKDIR/side_view.png"
convert "$WORKDIR/arrows_raw.png" -crop 600x600+340+120 +repage \
  -resize 480x480 -background '#d0d0d0' -gravity center \
  -extent 640x500 "$WORKDIR/arrow_view.png"
convert "$WORKDIR/side_view.png" "$WORKDIR/arrow_view.png" +append \
  "$WORKDIR/gazebo_view.png"
convert "$WORKDIR/r_hand_plot.png" -resize 340x210 "$WORKDIR/r_plot_view.png"
convert "$WORKDIR/l_hand_plot.png" -resize 340x210 "$WORKDIR/l_plot_view.png"
convert -size 1280x720 xc:white "$WORKDIR/gazebo_view.png" \
  -gravity northwest -geometry +0+0 -composite "$WORKDIR/r_plot_view.png" \
  -gravity northwest -geometry +290+505 -composite "$WORKDIR/l_plot_view.png" \
  -gravity northwest -geometry +650+505 -composite \
  -fill '#202020' -pointsize 16 -annotate +20+24 "Reference-matched side view" \
  -fill '#202020' -pointsize 16 -annotate +660+24 "Two-hand arrow evidence view" \
  -stroke '#777777' -strokewidth 2 -draw 'line 0,502 1280,502' \
  "$WORKDIR/current_view.png"
convert "$WORKDIR/current_view.png" -gravity north -background white -splice 0x40 \
  -fill black -pointsize 28 -annotate +20+8 "v2.0+ (Gazebo Harmonic)" "$WORKDIR/current.png"

convert "$LEGACY" -resize 1280x720 -gravity north -background white -splice 0x40 \
  -fill black -pointsize 28 -annotate +20+8 "2015 reference (Gazebo Classic)" "$WORKDIR/legacy.png"
convert "$WORKDIR/legacy.png" "$WORKDIR/current.png" +append -background white \
  -gravity center -append -bordercolor '#333333' -border 8 "$OUTPUT"

echo "Comparison saved to $OUTPUT"
echo "Raw camera proof saved to $RAW_PROOF"
echo "Raw arrow proof saved to $ARROW_PROOF"
echo "No-regression report saved to $REPORT"
ls -lh "$OUTPUT"
cat "$REPORT"
