#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
echo "启用 H743 MINI。定位和板端状态满足条件时，可能请求起立并前进。"
exec ros2 service call /automation/enable std_srvs/srv/SetBool '{data: true}'
