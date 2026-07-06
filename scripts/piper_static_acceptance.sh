#!/usr/bin/env bash
set -eo pipefail
# ROS/ament 的 setup 脚本会读取未定义变量；先显式关闭 nounset，环境加载完成后再打开。
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"

for piper_moveit_local_setup in \
  "${WORKSPACE_DIR}/external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_planners_ompl/local_setup.bash" \
  "${WORKSPACE_DIR}/external/ros_humble_debs/overlay/opt/ros/humble/share/moveit_simple_controller_manager/local_setup.bash"
do
  if [[ -f "${piper_moveit_local_setup}" ]]; then
    # 静态验收只加载 Piper 专用 MoveIt2 插件 overlay，不启动 move_group 或真实执行器。
    source "${piper_moveit_local_setup}"
  fi
done

set -u

echo "[Piper Static] 1/10 安全默认值检查..."
"${SCRIPT_DIR}/piper_safety_check.sh"

echo
echo "[Piper Static] 2/10 task1/Nav2 隔离边界检查..."
"${SCRIPT_DIR}/piper_boundary_check.sh"

echo
echo "[Piper Static] 3/10 GitHub 体积和数据产物边界检查..."
"${SCRIPT_DIR}/piper_repo_size_check.sh"

echo
echo "[Piper Static] 4/10 依赖和官方 Piper 包预检..."
"${SCRIPT_DIR}/piper_preflight.sh" --require-official

echo
echo "[Piper Static] 5/10 官方 URDF 到项目侧 piper_* frame 审计..."
"${SCRIPT_DIR}/piper_official_frame_audit.sh"

echo
echo "[Piper Static] 6/10 项目侧 MoveIt2 配置映射审计..."
"${SCRIPT_DIR}/piper_moveit_config_audit.sh"

echo
echo "[Piper Static] 7/10 手眼标定配置边界检查..."
"${SCRIPT_DIR}/piper_hand_eye_check.sh"

echo
echo "[Piper Static] 8/10 实机接入前状态报告..."
"${SCRIPT_DIR}/piper_real_readiness.sh"

echo
echo "[Piper Static] 9/10 Piper RViz 可视化配置烟测..."
"${SCRIPT_DIR}/piper_visualization_smoke.sh"

echo
echo "[Piper Static] 10/10 实机 dry-run launch 参数展开检查..."
ros2 launch slam_nav_piper_bringup piper_real.launch.py --show-args >/dev/null

echo
echo "[Piper Static] 静态配置验收通过：Piper 已配置、保持隔离，且未接入真实执行后端。"
