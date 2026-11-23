#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
echo "暂停 H743 MINI 并请求本会话收步。板端停机使用遥控 KEY7。"
exec ros2 service call /automation/enable std_srvs/srv/SetBool '{data: false}'
