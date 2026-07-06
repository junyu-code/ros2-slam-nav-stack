#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

mkdir -p src/slam_nav_bringup/map
name="${1:-nav_test_map}"
exec ros2 run nav2_map_server map_saver_cli -f "src/slam_nav_bringup/map/${name}"
