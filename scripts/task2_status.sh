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
  task2 长期扩展状态页，不启动 Gazebo、RViz、Nav2 或真实硬件。
  默认只检查文档、脚本、包目录和安全边界；追加 --with-preflight 时会顺带运行 real-preflight。

选项：
  --with-preflight  同时运行 ./run.sh real-preflight
  --strict          配合 --with-preflight 时传给 real-preflight；也会把 task1 未完成明确列为 WAIT
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
  if [[ -f "${path}" ]]; then
    ok "${desc}: ${path}"
  else
    fail "缺少 ${desc}: ${path}"
  fi
}

check_dir() {
  local path="$1"
  local desc="$2"
  if [[ -d "${path}" ]]; then
    ok "${desc}: ${path}"
  else
    fail "缺少 ${desc}: ${path}"
  fi
}

check_executable() {
  local path="$1"
  local desc="$2"
  if [[ -x "${path}" ]]; then
    ok "${desc}: ${path}"
  else
    fail "缺少或不可执行 ${desc}: ${path}"
  fi
}

contains_text() {
  local path="$1"
  local pattern="$2"
  local desc="$3"
  if [[ ! -f "${path}" ]]; then
    fail "无法检查 ${desc}，文件不存在: ${path}"
    return
  fi
  if grep -Eq "${pattern}" "${path}"; then
    ok "${desc}: ${path}"
  else
    warn "${desc} 未找到预期内容: ${path}"
  fi
}

echo "Task2 长期扩展状态"
echo "工作区: ${WORKSPACE_DIR}"
echo

echo "1. 文档骨架"
check_file "tasks/task2/FUTURE_ROADMAP.md" "长期路线图"
check_file "tasks/task2/ROBUST_NAVIGATION_UPGRADE_PLAN.md" "鲁棒导航升级计划"
check_file "tasks/task2/REAL_ROBOT_DEPLOYMENT_CHECKLIST.md" "实机部署检查清单"
check_file "tasks/task2/PIPER_MOBILE_MANIPULATION.md" "移动操作/机械臂路线"
contains_text "README.md" 'REAL_ROBOT_DEPLOYMENT_CHECKLIST|real-preflight|safe-bridge' "README 已指向实机预检/安全桥"
contains_text "PROJECT_PROCESS.md" 'REAL_ROBOT_DEPLOYMENT_CHECKLIST|task2|real-preflight' "过程记录包含 task2/实机部署维护记录"

echo
echo "2. 实机与鲁棒导航入口"
for script in \
  real_preflight.sh \
  start_safe_cmd_bridge.sh \
  start_localization_guard.sh \
  start_relocalization.sh \
  start_relocalization_gicp.sh \
  start_navigation_rgbd.sh \
  start_robust_navigation.sh \
  diagnose_runtime.sh; do
  check_executable "scripts/${script}" "脚本"
done

echo
echo "3. 扩展包边界"
for package_dir in \
  src/safe_cmd_bridge \
  src/localization_guard \
  src/cloud_relocalization \
  src/rgbd_navigation_perception \
  src/perception_adapter \
  src/mission_behavior \
  src/pb_nav2_plugins \
  src/pb_omni_pid_pursuit_controller \
  src/slam_nav_piper_bringup \
  src/slam_nav_piper_control \
  src/slam_nav_piper_perception \
  src/slam_nav_piper_manipulation \
  src/slam_nav_piper_calibration; do
  check_dir "${package_dir}" "扩展包目录"
done

echo
echo "4. 安全默认值与隔离边界"
contains_text "src/safe_cmd_bridge/config/safe_cmd_bridge.yaml" 'enable_udp_output:[[:space:]]*false' "UDP 输出默认关闭"
contains_text "src/safe_cmd_bridge/config/safe_cmd_bridge.yaml" 'command_timeout_sec' "速度指令超时配置存在"
contains_text "src/cloud_relocalization/config/icp_relocalization.yaml" 'publish_tf:[[:space:]]*false' "重定位默认不发布 map->odom"
contains_text "src/cloud_relocalization/config/icp_relocalization.yaml" 'auto_align:[[:space:]]*false' "重定位默认不自动闭环"
contains_text "src/rgbd_navigation_perception/README.md" '/nav_camera/d435i/depth/image_rect_raw' "导航 RGB-D 使用 D435i /nav_camera"
contains_text "src/slam_nav_piper_perception/README.md" '/piper/arm_camera' "机械臂相机保持 /piper/arm_camera"

if git ls-files | grep -Eq '(^|/)(.*\.(bag|db3|mcap|pcd|ply|las|laz|onnx|pt|pth|ckpt|engine|safetensors))$'; then
  warn "Git 中发现可能不应跟踪的重型数据/权重文件，请运行 ./run.sh task1-delivery-check 或 piper-size-check 复核"
else
  ok "Git 未跟踪 rosbag/点云/模型权重等重型产物"
fi

echo
echo "5. task1 与 task2 的衔接状态"
if "${SCRIPT_DIR}/task1_experiment_check.sh" --strict >/tmp/task2_status_task1_experiment.log 2>&1; then
  ok "task1 静态避障实验已满足 strict 成功率要求"
else
  wait_item "task1 静态避障实验 strict 尚未通过；先补真实 10 次记录和截图，再把精力切到实机闭环"
fi

if [[ -f "tasks/task1/TASK1_RUNTIME_LAST.md" ]]; then
  ok "task1 已有运行时 latest 快照"
else
  wait_item "task1 还没有运行时 latest 快照；跑建图/导航后执行 ./run.sh task1-runtime-check <mapping|nav> --save"
fi

if [[ -f "tasks/task1/report_latex/figures/fig_9_1_dynamic_obstacle.png" ]]; then
  ok "动态障碍扩展示范截图已补"
else
  wait_item "动态障碍扩展示范截图未补；task1 报告若写动态扩展，需要补图 9-1"
fi

echo
echo "6. 下一步建议"
if (( fail_count > 0 )); then
  echo "- 先修复 FAIL 项，再运行 ./run.sh task2-status。"
elif (( wait_count > 0 )); then
  echo "- 当前 task2 架构边界基本存在，但 task1 真实证据仍未补齐；建议先跑 task1 主流程。"
  echo "- task1 跑完后，再按 tasks/task2/REAL_ROBOT_DEPLOYMENT_CHECKLIST.md 做实机 dry-run。"
else
  echo "- 可以进入实机 dry-run：./run.sh real-preflight --strict"
  echo "- 然后按实机部署检查清单逐项接传感器、TF、safe-bridge 和重定位观测。"
fi

if [[ "${RUN_PREFLIGHT}" == "true" ]]; then
  echo
  echo "7. real-preflight"
  preflight_args=()
  if [[ "${STRICT}" == "true" ]]; then
    preflight_args+=(--strict)
  fi
  if "${SCRIPT_DIR}/real_preflight.sh" "${preflight_args[@]}"; then
    ok "real-preflight 执行完成"
  else
    fail "real-preflight 未通过"
  fi
fi

echo
echo "汇总: OK=${ok_count}, WAIT=${wait_count}, WARN=${warn_count}, FAIL=${fail_count}"

if (( fail_count > 0 )); then
  exit 1
fi

if [[ "${STRICT}" == "true" && ( "${wait_count}" -gt 0 || "${warn_count}" -gt 0 ) ]]; then
  echo "[task2-status] --strict 模式下 WAIT/WARN 也表示仍未进入实机闭环完成态。" >&2
  exit 1
fi

exit 0
