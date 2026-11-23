#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
export PYTHONPATH="$PROJECT_ROOT/lidar_ws/src/ros2_to_stm32:${PYTHONPATH:-}"
exec python3 -m ros2_to_stm32.h7_mini "$@"
