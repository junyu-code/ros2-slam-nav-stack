#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"

for piper_moveit_local_setup in \
  "${WORKSPACE_DIR}/external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_planners_ompl/local_setup.bash" \
  "${WORKSPACE_DIR}/external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_simple_controller_manager/local_setup.bash"
do
  if [[ -f "${piper_moveit_local_setup}" ]]; then
    # 全链路烟测同样加载 Piper 专用 MoveIt2 本地插件，保持无需 sudo 的验证路径。
    source "${piper_moveit_local_setup}"
  fi
done

set -u

echo "[Piper Full] 1/13 安全配置检查..."
"${SCRIPT_DIR}/piper_safety_check.sh"

echo
echo "[Piper Full] 2/13 边界回归检查..."
"${SCRIPT_DIR}/piper_boundary_check.sh"

echo
echo "[Piper Full] 3/13 依赖预检和官方包检查..."
"${SCRIPT_DIR}/piper_preflight.sh" --require-official

echo
echo "[Piper Full] 4/13 官方 URDF -> 项目侧 piper_* frame 审计..."
ros2 run slam_nav_piper_bringup piper_official_frame_audit.py --check-project-adapter

echo
echo "[Piper Full] 5/13 项目侧 MoveIt2 配置映射审计..."
"${SCRIPT_DIR}/piper_moveit_config_audit.sh"

echo
echo "[Piper Full] 6/13 Piper 运行时 TF 链烟测..."
"${SCRIPT_DIR}/piper_tf_smoke.sh"

echo
echo "[Piper Full] 7/13 Piper runtime 命名空间边界烟测..."
"${SCRIPT_DIR}/piper_namespace_smoke.sh"

echo
echo "[Piper Full] 8/13 Piper 控制桥安全边界烟测..."
"${SCRIPT_DIR}/piper_control_smoke.sh"

echo
echo "[Piper Full] 9/13 Piper 实机入口 dry-run 安全拒绝烟测..."
"${SCRIPT_DIR}/piper_real_dry_run.sh"

echo
echo "[Piper Full] 10/13 Headless Gazebo 官方 Piper 组合模型烟测..."
"${SCRIPT_DIR}/piper_gazebo_smoke.sh"

echo
echo "[Piper Full] 11/13 Fake RGB-D -> grasp candidates -> pick/place action 烟测..."
"${SCRIPT_DIR}/piper_task_smoke.sh"

echo
echo "[Piper Full] 12/13 Piper 学习层抓取候选排序烟测..."
"${SCRIPT_DIR}/piper_learning_smoke.sh"

echo
echo "[Piper Full] 13/13 MoveIt2 plan-only 烟测..."
ROS_DOMAIN_ID="${PIPER_FULL_SMOKE_MOVEIT_DOMAIN_ID:-81}" \
  "${SCRIPT_DIR}/piper_moveit_smoke.sh"

echo
echo "[Piper Full] Piper 全链路烟测通过。"
