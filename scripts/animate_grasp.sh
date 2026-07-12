#!/usr/bin/env bash
# Run the modern task-space grasp controller for the release demo.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$ROOT/scripts/grasp_task_controller.py" "$@"
