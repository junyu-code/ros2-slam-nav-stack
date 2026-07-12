#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

RUN_PREFLIGHT=false
STRICT=false

usage() {
  cat <<'EOF'
用法：
  ./run.sh task2-status [--with-preflight] [--strict]

说明：
  按“代码、静态检查、仿真、实机”四个层级报告 Task2 状态。
  Task1 完成前 Task2 保持冻结；文档或目录存在不会被当成功能完成。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-preflight)
      RUN_PREFLIGHT=true
      ;;
    --strict)
      STRICT=true
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task2-status] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

source "${SCRIPT_DIR}/task1_state.sh"

ok_count=0
wait_count=0
warn_count=0
fail_count=0

ok() {
  ok_count=$((ok_count + 1))
  echo "[OK] $*"
}

wait_item() {
  wait_count=$((wait_count + 1))
  echo "[WAIT] $*"
}

warn() {
  warn_count=$((warn_count + 1))
  echo "[WARN] $*"
}

fail() {
  fail_count=$((fail_count + 1))
  echo "[FAIL] $*"
}

check_file() {
  local path="$1"
  local desc="$2"
  [[ -f "${path}" ]] && ok "${desc}: ${path}" || fail "缺少 ${desc}: ${path}"
}

check_dir() {
  local path="$1"
  local desc="$2"
  [[ -d "${path}" ]] && ok "${desc}: ${path}" || fail "缺少 ${desc}: ${path}"
}

check_executable() {
  local path="$1"
  [[ -x "${path}" ]] && ok "入口可执行: ${path}" || fail "入口缺少或不可执行: ${path}"
}

contains_text() {
  local path="$1"
  local pattern="$2"
  local desc="$3"
  if [[ -f "${path}" ]] && grep -Eq "${pattern}" "${path}"; then
    ok "${desc}"
  else
    fail "${desc}"
  fi
}

echo "Task2 冻结状态"
echo "工作区: ${WORKSPACE_DIR}"

echo
echo "1. 冻结边界"
if task1_load_state; then
  task1_print_state | sed 's/^/  /'
  if task1_state_is_ready; then
    wait_item "Task1 已 ready；Task2 可以由维护者明确解冻，但当前仍未自动解冻"
  else
    wait_item "Task1 尚未完成，Task2 保持冻结"
  fi
else
  fail "无法读取 Task1 状态"
fi
check_file "tasks/task2/README.md" "Task2 能力矩阵"
check_file "tasks/task2/REAL_ROBOT_RUNBOOK.md" "实机 Runbook"

echo
echo "2. 代码存在"
task2_scripts=(
  real_preflight.sh start_safe_cmd_bridge.sh start_localization_guard.sh
  start_relocalization.sh start_relocalization_gicp.sh
  start_navigation_3d.sh start_navigation_rgbd.sh start_robust_navigation.sh
  diagnose_runtime.sh
)
for script in "${task2_scripts[@]}"; do
  check_executable "scripts/${script}"
done

task2_packages=(
  src/safe_cmd_bridge src/localization_guard src/cloud_relocalization
  src/rgbd_navigation_perception src/perception_adapter src/mission_behavior
  src/pb_nav2_plugins src/pb_omni_pid_pursuit_controller
  src/slam_nav_piper_bringup src/slam_nav_piper_control
  src/slam_nav_piper_perception src/slam_nav_piper_manipulation
  src/slam_nav_piper_calibration
)
for package_dir in "${task2_packages[@]}"; do
  check_dir "${package_dir}" "扩展包"
done

echo
echo "3. 静态安全检查"
contains_text "src/safe_cmd_bridge/config/safe_cmd_bridge.yaml" 'enable_udp_output:[[:space:]]*false' "UDP 输出默认关闭"
contains_text "src/safe_cmd_bridge/config/safe_cmd_bridge.yaml" 'command_timeout_sec' "速度命令超时配置存在"
contains_text "src/cloud_relocalization/config/icp_relocalization.yaml" 'publish_tf:[[:space:]]*false' "重定位默认不发布 map->odom"
contains_text "src/cloud_relocalization/config/icp_relocalization.yaml" 'auto_align:[[:space:]]*false' "重定位默认不自动闭环"
contains_text "src/rgbd_navigation_perception/README.md" '/nav_camera/depth/image_raw' "导航 RGB-D 命名空间已隔离"
contains_text "src/slam_nav_piper_perception/README.md" '/piper/arm_camera' "Piper 相机命名空间已隔离"

echo
echo "4. 仿真验证"
if [[ -f "tasks/task1/report_latex/figures/fig_9_2_rgbd_visual_obstacles.png" ]]; then
  ok "RGB-D 有阶段性仿真截图，但仍需按冻结方案复测"
else
  wait_item "RGB-D 尚无阶段性仿真截图"
fi
if [[ -f "tasks/task1/report_latex/figures/fig_9_1_dynamic_obstacle.png" ]]; then
  ok "动态障碍有阶段性仿真截图，但不代表稳定闭环"
else
  wait_item "动态障碍尚无阶段性仿真截图"
fi
wait_item "3D 地形、自由空间恢复和重定位仍缺统一条件下的重复对比实验"

echo
echo "5. 实机验证"
wait_item "真实 MID360/IMU/D435i 话题与外参尚未现场验收"
wait_item "真实底盘协议、方向、反馈看门狗和急停尚未现场验收"
wait_item "Piper 手眼标定、真实 MoveIt 执行和 SDK 后端尚未现场验收"

if [[ "${RUN_PREFLIGHT}" == "true" ]]; then
  echo
  echo "6. real-preflight"
  preflight_args=()
  [[ "${STRICT}" == "true" ]] && preflight_args+=(--strict)
  if "${SCRIPT_DIR}/real_preflight.sh" "${preflight_args[@]}"; then
    ok "real-preflight 执行完成"
  else
    fail "real-preflight 未通过"
  fi
fi

echo
echo "下一步:"
echo "- 先完成 Task1 导航方案选择和正式实验。"
echo "- Task2 解冻后，从 REAL_ROBOT_RUNBOOK.md 的无硬件预检开始，不能直接打开 UDP。"

echo
echo "汇总: OK=${ok_count}, WAIT=${wait_count}, WARN=${warn_count}, FAIL=${fail_count}"
if (( fail_count > 0 )); then
  exit 1
fi
if [[ "${STRICT}" == "true" && ( "${wait_count}" -gt 0 || "${warn_count}" -gt 0 ) ]]; then
  echo "[task2-status] --strict 模式下 WAIT/WARN 表示尚未完成。" >&2
  exit 1
fi
