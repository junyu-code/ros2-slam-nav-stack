#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 只控制碰撞测试用的手动车，不占用自动导航车 /cmd_vel。
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/manual_car/cmd_vel
