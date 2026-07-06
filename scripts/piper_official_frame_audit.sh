#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

# 默认检查项目侧 piper_* 适配链，确认没有退回官方原生 base_link/link*/joint* 或占位关节链。
exec ros2 run slam_nav_piper_bringup piper_official_frame_audit.py --check-project-adapter "$@"
