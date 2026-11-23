#!/usr/bin/env bash
set -eo pipefail
RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

check_environment() {
    printf '系统架构：%s\n' "$(uname -m)"
    if [[ ! -r /opt/ros/humble/setup.bash ]]; then
        echo "没有找到 /opt/ros/humble/setup.bash。请确认原项目使用 ROS 2 Humble，或查看 Nano 部署说明。" >&2
        return 1
    fi
    source /opt/ros/humble/setup.bash
    for tool in cmake colcon c++ pkg-config; do
        command -v "$tool" >/dev/null || { echo "缺少命令：$tool" >&2; return 1; }
    done
    python3 - <<'PYTHON'
import sys

if sys.version_info < (3, 10):
    raise SystemExit('需要 Python 3.10 或更新版本')

import rclpy
import serial
from ament_index_python.packages import get_package_prefix, PackageNotFoundError

packages = ('ament_cmake_auto', 'pcl_ros', 'pcl_conversions', 'rosidl_default_generators')
for package in packages:
    try:
        get_package_prefix(package)
    except PackageNotFoundError:
        raise SystemExit(f'ROS 2 Humble 缺少包：{package}')
PYTHON
    if ! pkg-config --exists apr-1; then
        echo "缺少 APR 开发库，请检查 libapr1-dev。" >&2
        return 1
    fi
    echo "H7 MINI 原生编译环境检查通过。"
}

case "${1:-}" in
    check)
        check_environment
        ;;
    build)
        check_environment
        BUILD_PROFILE=mini BUILD_JOBS="${BUILD_JOBS:-1}" bash "$RUN_DIR/build_ros2.sh"
        ;;
    run)
        shift
        source "$RUN_DIR/environment.sh"
        exec ros2 launch ros2_to_stm32 h7_nano.launch.py "$@"
        ;;
    status)
        shift
        exec bash "$RUN_DIR/h7_mini.sh" --status "$@"
        ;;
    arm|pause|reset_route)
        command="$1"
        shift
        exec bash "$RUN_DIR/$command.sh" "$@"
        ;;
    *)
        echo "用法：bash run/nano.sh {check|build|run|status|arm|pause|reset_route}" >&2
        exit 2
        ;;
esac
