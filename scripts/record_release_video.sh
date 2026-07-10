#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"
WORLD_FILE="${WORLD_FILE:-$ROOT/worlds/release_demo.world}"
OUTPUT_VIDEO="${OUTPUT_VIDEO:-$BUILD_DIR/release_demo.mp4}"
DISPLAY_NUM="${DISPLAY_NUM:-99}"
VIDEO_SIZE="${VIDEO_SIZE:-1280x720}"
RECORD_SECONDS="${RECORD_SECONDS:-14}"

export GZ_SIM_SYSTEM_PLUGIN_PATH="${BUILD_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${ROOT}/models:${GZ_SIM_RESOURCE_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

mkdir -p "$(dirname "$OUTPUT_VIDEO")"

if [[ ! -f "$BUILD_DIR/libvectorview.so" ]]; then
  echo "error: build libvectorview.so first (cmake --build build)" >&2
  exit 1
fi

cleanup() {
  jobs -p | xargs -r kill 2>/dev/null || true
  if [[ -n "${XVFB_PID:-}" ]]; then kill "$XVFB_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT

echo "Starting virtual display :${DISPLAY_NUM}..."
Xvfb ":${DISPLAY_NUM}" -screen 0 "${VIDEO_SIZE}x24" +extension GLX +render -noreset &
XVFB_PID=$!
export DISPLAY=":${DISPLAY_NUM}"
sleep 2

echo "Launching Gazebo Sim release demo..."
gz sim -r "$WORLD_FILE" &
SIM_PID=$!
sleep 10

# Wait until the world is accepting pose commands.
for _ in $(seq 1 30); do
  if gz service -l 2>/dev/null | grep -q "/world/release_demo/set_pose"; then
    break
  fi
  sleep 1
done

echo "Recording ${RECORD_SECONDS}s to ${OUTPUT_VIDEO}..."
ffmpeg -y -loglevel error \
  -f x11grab -draw_mouse 0 -video_size "${VIDEO_SIZE}" -framerate 30 \
  -i "${DISPLAY}.0" -t "${RECORD_SECONDS}" \
  -c:v libx264 -preset veryfast -pix_fmt yuv420p \
  "$OUTPUT_VIDEO" &
FFMPEG_PID=$!
sleep 1

"${ROOT}/scripts/animate_release_demo.sh"

wait "$FFMPEG_PID"
kill "$SIM_PID" 2>/dev/null || true
wait "$SIM_PID" 2>/dev/null || true

if [[ ! -s "$OUTPUT_VIDEO" ]]; then
  echo "error: video was not created" >&2
  exit 1
fi

echo "Saved release demo video: $OUTPUT_VIDEO"
ls -lh "$OUTPUT_VIDEO"
