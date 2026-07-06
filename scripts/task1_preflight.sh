#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

strict=false
if [[ "${1:-}" == "--strict" ]]; then
  strict=true
fi

errors=0
warnings=0

ok() {
  echo "[task1-check] OK: $*"
}

warn() {
  warnings=$((warnings + 1))
  echo "[task1-check] WARN: $*" >&2
}

fail() {
  errors=$((errors + 1))
  echo "[task1-check] FAIL: $*" >&2
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

check_executable() {
  local path="$1"
  local desc="$2"
  if [[ -x "${path}" ]]; then
    ok "${desc}: ${path}"
  else
    fail "缺少或不可执行 ${desc}: ${path}"
  fi
}

check_optional_image() {
  local path="$1"
  local desc="$2"
  if [[ -f "${path}" ]]; then
    ok "${desc}: ${path}"
  else
    warn "待补截图 ${desc}: ${path}"
  fi
}

echo "[task1-check] 工作区: ${WORKSPACE_DIR}"

# 根目录只保留 run.sh 作为用户入口，具体脚本集中在 scripts/。
check_executable "run.sh" "根目录统一入口"
for stale in start_simulation.sh start_mapping.sh start_navigation.sh start_navigation_3d.sh setup_piper_open_class.sh; do
  if [[ -e "${stale}" ]]; then
    fail "根目录仍存在旧 wrapper，应改用 ./run.sh: ${stale}"
  else
    ok "根目录旧 wrapper 已清理: ${stale}"
  fi
done

for script in \
  build.sh clean.sh start_simulation_static.sh start_simulation_dynamic.sh \
  start_mapping.sh start_auto_mapping.sh teleop.sh save_map.sh \
  start_navigation.sh start_navigation_3d.sh start_navigation_full.sh \
  diagnose_runtime.sh; do
  check_executable "scripts/${script}" "脚本"
done

help_text="$(./run.sh help || true)"
for command_name in sim-static sim-dynamic mapping auto-mapping teleop save-map nav nav-3d nav-full diagnose task1-check; do
  if grep -q "${command_name}" <<<"${help_text}"; then
    ok "run.sh help 包含命令: ${command_name}"
  else
    fail "run.sh help 缺少命令: ${command_name}"
  fi
done

# 核心代码和地图文件。
for package_dir in \
  src/slam_nav_simulation \
  src/slam_nav_bringup \
  src/FAST_LIO \
  src/pointcloud_to_laserscan \
  src/perception_adapter \
  src/cloud_relocalization; do
  if [[ -d "${package_dir}" ]]; then
    ok "核心包目录存在: ${package_dir}"
  else
    fail "核心包目录缺失: ${package_dir}"
  fi
done

check_file "src/slam_nav_bringup/map/nav_test_map.yaml" "默认导航地图 yaml"
check_file "src/slam_nav_bringup/map/nav_test_map.pgm" "默认导航地图 pgm"
check_file "src/slam_nav_simulation/world/nav_test_world/nav_test_world.world" "静态仿真场地"
check_file "src/slam_nav_simulation/world/nav_test_world/nav_test_world_dynamic.world" "动态障碍物仿真场地"

# task1 文档链路。
for doc in \
  tasks/task1/TASK1_FINAL_RUNBOOK.md \
  tasks/task1/RUN_AND_SCREENSHOT_STEPS.md \
  tasks/task1/EXPERIMENT_RECORD.md \
  tasks/task1/DELIVERY_CHECKLIST.md \
  tasks/task1/SLAM_FINAL_REPORT_DRAFT.md \
  tasks/task1/report_latex/main.tex; do
  check_file "${doc}" "task1 文档"
done

check_file "tasks/task1/homework_latex/main.tex" "平时作业 LaTeX 源文件"
if [[ -f "tasks/task1/homework_latex/main.pdf" ]]; then
  ok "平时作业 PDF 已生成"
else
  warn "平时作业 PDF 未生成或不在本地"
fi

if [[ -f "tasks/task1/report_latex/main.pdf" ]]; then
  ok "结课报告 PDF 已生成"
else
  warn "结课报告 PDF 未生成或不在本地；可在 Windows TeX Live 中重新编译 main.tex"
fi

# 必需截图由用户 GUI 跑完后补齐，默认作为 warning。
FIG_DIR="tasks/task1/report_latex/figures"
mkdir -p "${FIG_DIR}"
check_optional_image "${FIG_DIR}/fig_6_1_gazebo_world.png" "图 6-1 Gazebo 静态场地总览"
check_optional_image "${FIG_DIR}/fig_6_2_robot_model.png" "图 6-2 机器人模型和传感器"
check_optional_image "${FIG_DIR}/fig_7_1_mapping_rviz.png" "图 7-1 RViz 建图过程"
check_optional_image "${FIG_DIR}/fig_7_2_saved_map.png" "图 7-2 保存后的地图"
check_optional_image "${FIG_DIR}/fig_8_1_nav2_map_loaded.png" "图 8-1 Nav2 加载地图"
check_optional_image "${FIG_DIR}/fig_8_2_global_path.png" "图 8-2 全局路径"
check_optional_image "${FIG_DIR}/fig_8_3_avoid_obstacle.png" "图 8-3 静态避障过程"
check_optional_image "${FIG_DIR}/fig_8_4_goal_reached.png" "图 8-4 到达目标点"
check_optional_image "${FIG_DIR}/fig_9_1_dynamic_obstacle.png" "图 9-1 动态障碍物扩展演示"

# 这些占位内容需要在最终提交前人工替换。
todo_count="$(grep -RInE '待填|待补|待替换|【待插图|placeholderfigure' \
  tasks/task1/EXPERIMENT_RECORD.md \
  tasks/task1/SLAM_FINAL_REPORT_DRAFT.md \
  tasks/task1/report_latex/main.tex 2>/dev/null | wc -l | tr -d ' ')"
if [[ "${todo_count}" == "0" ]]; then
  ok "task1 实验记录和报告中未发现待填/待替换占位"
else
  warn "task1 实验记录或报告中仍有 ${todo_count} 处待填/待替换占位"
fi

# 避免报告里残留无关比赛业务字段。
competition_hits="$(grep -RInE 'RoboMaster|RMUC|哨兵|云台|比赛模式|比赛业务' \
  tasks/task1/SLAM_FINAL_REPORT_DRAFT.md \
  tasks/task1/report_latex/main.tex 2>/dev/null || true)"
if [[ -z "${competition_hits}" ]]; then
  ok "task1 报告未发现明显无关比赛业务字段"
else
  warn "task1 报告可能仍有无关比赛业务字段："
  echo "${competition_hits}" >&2
fi

echo "[task1-check] 结构错误: ${errors}, 提交前提醒: ${warnings}"

if [[ "${errors}" -ne 0 ]]; then
  exit 1
fi

if [[ "${strict}" == "true" && "${warnings}" -ne 0 ]]; then
  echo "[task1-check] --strict 模式下 warning 也视为未完成。" >&2
  exit 1
fi

exit 0
