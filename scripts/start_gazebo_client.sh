#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 单独打开 Gazebo 图形客户端；仿真后端已由 sim-static/headless 启动。
exec gzclient --verbose "$@"
