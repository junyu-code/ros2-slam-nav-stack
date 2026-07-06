#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

source "${SCRIPT_DIR}/setup_workspace_env.sh"

for piper_moveit_local_setup in \
  "${SCRIPT_DIR}/../external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_planners_ompl/local_setup.bash" \
  "${SCRIPT_DIR}/../external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_simple_controller_manager/local_setup.bash"
do
  if [[ -f "${piper_moveit_local_setup}" ]]; then
    # 可视化入口允许显式打开 plan-only MoveIt2，因此顺手加载本地规划插件 overlay。
    source "${piper_moveit_local_setup}"
  fi
done

set -u
exec ros2 launch slam_nav_piper_bringup piper_visualization.launch.py "$@"
