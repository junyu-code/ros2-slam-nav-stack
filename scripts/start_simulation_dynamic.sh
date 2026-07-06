#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"

# 动态障碍物仿真统一复用主启动脚本，避免两套 ROS 环境初始化逻辑不一致。
exec "${SCRIPT_DIR}/start_simulation.sh" "$@" world:=dynamic
