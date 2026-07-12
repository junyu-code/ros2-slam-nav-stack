#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WORKSPACE_DIR}"
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

export SLAM_NAV_WS="${WORKSPACE_DIR}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[operator] 未找到 ros2，请检查 ROS2 环境。" >&2
  exit 2
fi

if ! ros2 pkg prefix slam_nav_operator >/dev/null 2>&1; then
  echo "[operator] slam_nav_operator 尚未构建，先运行 ./run.sh build。" >&2
  exit 2
fi

exec ros2 run slam_nav_operator slam_nav_operator "$@"
