#!/usr/bin/env bash
set -uo pipefail
export VECTOR_VIEW=/workspace
export GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/build
export GZ_SIM_RESOURCE_PATH=/workspace/assets/models
export GZ_SIM_USER_PATH=/workspace/assets/worlds

killall -9 gz 2>/dev/null || true
sleep 2
DISPLAY= gz sim -s -r --headless-rendering /workspace/assets/worlds/release_demo.world > /tmp/gz_test.log 2>&1 &
sleep 14
/workspace/scripts/animate_grasp.sh > /tmp/anim.log 2>&1 &
ANIM_PID=$!
sleep 8.5
echo "=== L HAND ===" > /tmp/contact_test.txt
timeout 6 gz topic -e -t /vectorview/iCub_fixed/l_hand -n 1 >> /tmp/contact_test.txt 2>&1 || true
echo "=== R HAND ===" >> /tmp/contact_test.txt
timeout 6 gz topic -e -t /vectorview/iCub_fixed/r_hand -n 1 >> /tmp/contact_test.txt 2>&1 || true
wait "$ANIM_PID" || true
killall -9 gz 2>/dev/null || true
grep 'collision' /tmp/contact_test.txt | sort -u
