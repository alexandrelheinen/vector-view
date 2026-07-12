#!/usr/bin/env bash
# Build VectorView, run the release demo world headlessly, record a grasp video.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/grasp_demo.mp4}"
WORLD="$ROOT/assets/worlds/release_demo.world"
RECORD_TOPIC="/camera/record_video"
SIM_LOG="/tmp/gz_sim_release_demo.log"

export VECTOR_VIEW="$ROOT"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/assets/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_USER_PATH="$ROOT/assets/worlds:${GZ_SIM_USER_PATH:-}"

if [ ! -f "$ROOT/build/libvector-view.so" ]; then
  echo "error: build/libvector-view.so not found. Build the project first." >&2
  exit 1
fi

rm -f "$OUTPUT"

echo "Starting Gazebo Sim (headless) with $WORLD"
DISPLAY= setsid gz sim -s -r --headless-rendering "$WORLD" >"$SIM_LOG" 2>&1 &
SIM_PID=$!

stop_sim() {
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    return 0
  fi

  # Do not call WorldControl shutdown here: it can crash the video recorder
  # before the MP4 is finalized. Send signals to the whole session instead.
  kill -TERM -- -"$SIM_PID" 2>/dev/null || kill -TERM "$SIM_PID" 2>/dev/null || true
  for _ in $(seq 1 15); do
    kill -0 "$SIM_PID" 2>/dev/null || return 0
    sleep 1
  done

  kill -KILL -- -"$SIM_PID" 2>/dev/null || kill -KILL "$SIM_PID" 2>/dev/null || true
  for _ in $(seq 1 5); do
    kill -0 "$SIM_PID" 2>/dev/null || return 0
    sleep 1
  done
}

cleanup() {
  stop_sim
}
trap cleanup EXIT

echo "Waiting for simulation and recorder service..."
ready=0
for _ in $(seq 1 90); do
  if gz service -l 2>/dev/null | grep -q "$RECORD_TOPIC"; then
    ready=1
    break
  fi
  if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "error: gz sim exited early. Log:" >&2
    tail -n 80 "$SIM_LOG" >&2 || true
    exit 1
  fi
  sleep 1
done

if [ "$ready" -ne 1 ]; then
  echo "error: recorder service $RECORD_TOPIC did not appear in time." >&2
  tail -n 80 "$SIM_LOG" >&2 || true
  exit 1
fi

# Let joint controllers settle into the standing pose before recording.
sleep 5

echo "Recording to $OUTPUT"
gz service -s "$RECORD_TOPIC" \
  --reqtype gz.msgs.VideoRecord \
  --reptype gz.msgs.Boolean \
  --timeout 10000 \
  --req "start: true, format: 'mp4', save_filename: '${OUTPUT}'"

"$ROOT/scripts/animate_grasp.sh"

# Brief pause so the last contact arrows are captured.
sleep 1

echo "Stopping recording"
gz service -s "$RECORD_TOPIC" \
  --reqtype gz.msgs.VideoRecord \
  --reptype gz.msgs.Boolean \
  --timeout 30000 \
  --req "stop: true"

REPORT="${OUTPUT%.*}-report.json"

# Keep gz sim alive while the recorder finalizes the MP4 on disk.
for _ in $(seq 1 90); do
  if [ -f "$OUTPUT" ] && [ -s "$OUTPUT" ]; then
    break
  fi
  sleep 1
done

if [ ! -f "$OUTPUT" ] || [ ! -s "$OUTPUT" ]; then
  echo "error: video file was not created at $OUTPUT" >&2
  tail -n 80 "$SIM_LOG" >&2 || true
  exit 1
fi

echo "Video saved: $OUTPUT ($(du -h "$OUTPUT" | awk '{print $1}'))"
python3 "$ROOT/scripts/verify_release_video.py" "$OUTPUT" --report "$REPORT"

trap - EXIT
stop_sim
