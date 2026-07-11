#!/usr/bin/env bash
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export VECTOR_VIEW="$ROOT"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/assets/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_USER_PATH="$ROOT/assets/worlds:${GZ_SIM_USER_PATH:-}"

killall -9 gz 2>/dev/null || true
sleep 2
DISPLAY= gz sim -s -r --headless-rendering "$ROOT/assets/worlds/release_demo.world" > /tmp/gz_test.log 2>&1 &
sleep 14
"$ROOT/scripts/animate_grasp.sh" > /tmp/anim.log 2>&1 &
ANIM_PID=$!
sleep 8.5
echo "=== L HAND ===" > /tmp/contact_test.txt
timeout 6 gz topic -e -t /vectorview/iCub_fixed/l_hand -n 1 >> /tmp/contact_test.txt 2>&1 || true
echo "=== R HAND ===" >> /tmp/contact_test.txt
timeout 6 gz topic -e -t /vectorview/iCub_fixed/r_hand -n 1 >> /tmp/contact_test.txt 2>&1 || true
python3 "$ROOT/scripts/save_camera_frame.py" --output /tmp/contact_capture.png --timeout 15
wait "$ANIM_PID" || true
killall -9 gz 2>/dev/null || true
echo "=== COLLISIONS ==="
grep 'name:' /tmp/contact_test.txt | sort -u
python3 - <<'PY'
from PIL import Image
import numpy as np
img = np.array(Image.open('/tmp/contact_capture.png'))
blue = (img[:,:,2] > 120) & (img[:,:,2] > img[:,:,0]+30) & (img[:,:,2] > img[:,:,1]+20)
print('blue_pixels', int(blue.sum()))
PY
