#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

FIG_DIR="tasks/task1/report_latex/figures"
REPORT_TEX="tasks/task1/report_latex/main.tex"
REPORT_PDF="tasks/task1/report_latex/main.pdf"
EXPERIMENT_RECORD="tasks/task1/EXPERIMENT_RECORD.md"
REPORT_DRAFT="tasks/task1/SLAM_FINAL_REPORT_DRAFT.md"
RUNTIME_SNAPSHOT="tasks/task1/TASK1_RUNTIME_LAST.md"

required_figs=(
  "fig_6_1_gazebo_world.png|图 6-1 Gazebo 静态场地总览|./run.sh clean && ./run.sh sim-static"
  "fig_6_2_robot_model.png|图 6-2 机器人模型和传感器|./run.sh sim-static"
  "fig_7_1_mapping_rviz.png|图 7-1 RViz 建图过程|./run.sh mapping + ./run.sh teleop"
  "fig_7_2_saved_map.png|图 7-2 保存后的地图|./run.sh save-map nav_test_map"
  "fig_8_1_nav2_map_loaded.png|图 8-1 Nav2 加载地图|./run.sh nav"
  "fig_8_2_global_path.png|图 8-2 全局路径|RViz Nav2 Goal"
  "fig_8_3_avoid_obstacle.png|图 8-3 静态避障过程|RViz 导航过程中截图"
  "fig_8_4_goal_reached.png|图 8-4 到达目标点|RViz 到达目标后截图"
  "fig_9_1_dynamic_obstacle.png|图 9-1 动态障碍物扩展示范|./run.sh sim-dynamic + ./run.sh nav-3d"
)

optional_figs=(
  "fig_8_5_backup_recovery.png|图 8-5 后退恢复过程"
  "fig_9_2_rgbd_visual_obstacles.png|图 9-2 RGB-D 近场障碍点云"
  "fig_9_3_perception_adapter.png|图 9-3 感知接口说明"
)

pass_count=0
warn_count=0
fail_count=0

pass() {
  pass_count=$((pass_count + 1))
  echo "[OK] $*"
}

warn() {
  warn_count=$((warn_count + 1))
  echo "[TODO] $*"
}

fail() {
  fail_count=$((fail_count + 1))
  echo "[缺失] $*"
}

file_size() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    stat -c '%s bytes' "${path}"
  else
    echo "missing"
  fi
}

file_mtime() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    date -d "@$(stat -c '%Y' "${path}")" '+%Y-%m-%d %H:%M:%S'
  else
    echo "missing"
  fi
}

count_matches() {
  local pattern="$1"
  shift
  grep -RInE "${pattern}" "$@" 2>/dev/null | wc -l | tr -d ' '
}

echo "Task1 当前状态"
echo "工作区: ${WORKSPACE_DIR}"
echo

echo "1. 基础文件"
if [[ -f "run.sh" && -x "run.sh" ]]; then
  pass "统一入口 run.sh 可执行"
else
  fail "run.sh 缺失或不可执行"
fi

if [[ -f "src/slam_nav_bringup/map/nav_test_map.yaml" && -f "src/slam_nav_bringup/map/nav_test_map.pgm" ]]; then
  pass "默认导航地图存在: nav_test_map.yaml ($(file_size src/slam_nav_bringup/map/nav_test_map.yaml)), nav_test_map.pgm ($(file_size src/slam_nav_bringup/map/nav_test_map.pgm))"
else
  fail "默认导航地图不完整，先完成建图并执行 ./run.sh save-map nav_test_map"
fi

if [[ -f "${REPORT_PDF}" ]]; then
  pass "结课报告 PDF 存在: ${REPORT_PDF}，更新时间 $(file_mtime "${REPORT_PDF}")"
else
  warn "结课报告 PDF 不存在，补齐截图后执行 ./run.sh task1-build-report"
fi

if [[ -f "${RUNTIME_SNAPSHOT}" ]]; then
  pass "运行时检查快照存在: ${RUNTIME_SNAPSHOT}，更新时间 $(file_mtime "${RUNTIME_SNAPSHOT}")"
else
  warn "运行时检查快照尚未生成；启动建图或导航后可运行 ./run.sh task1-runtime-check mapping --save 或 ./run.sh task1-runtime-check nav --save"
fi

echo
echo "2. 必需截图"
missing_figs=()
for item in "${required_figs[@]}"; do
  IFS='|' read -r file desc hint <<<"${item}"
  path="${FIG_DIR}/${file}"
  if [[ -f "${path}" ]]; then
    pass "${desc}: ${path}"
  else
    warn "${desc}: 缺 ${path}，建议步骤: ${hint}"
    missing_figs+=("${file}")
  fi
done

echo
echo "3. 可选扩展截图"
for item in "${optional_figs[@]}"; do
  IFS='|' read -r file desc <<<"${item}"
  path="${FIG_DIR}/${file}"
  if [[ -f "${path}" ]]; then
    pass "${desc}: ${path}"
  else
    echo "[可选] ${desc}: 未补 ${path}"
  fi
done

echo
echo "4. 实验记录与报告占位"
todo_total="$(count_matches '待填|待补|待替换|【待插图|placeholderfigure' "${EXPERIMENT_RECORD}" "${REPORT_DRAFT}" "${REPORT_TEX}")"
experiment_pending="$(grep -o '待填' "${EXPERIMENT_RECORD}" 2>/dev/null | wc -l | tr -d ' ')"
if [[ "${todo_total}" == "0" ]]; then
  pass "实验记录和报告正文没有待填/待替换占位"
else
  warn "实验记录或报告仍有 ${todo_total} 处待填/待替换占位"
fi

if [[ "${experiment_pending}" == "0" ]]; then
  pass "EXPERIMENT_RECORD.md 没有待填字段"
else
  warn "EXPERIMENT_RECORD.md 仍有 ${experiment_pending} 个待填字段，重点补 10 次静态避障结果"
fi

echo
echo "5. Git 状态"
if git diff --quiet && git diff --cached --quiet; then
  pass "Git 工作区干净"
else
  warn "Git 工作区存在未提交改动，最终打包前建议先确认并提交"
  git status --short
fi

echo
echo "6. 下一步建议"
if (( fail_count > 0 )); then
  echo "- 先修复上面的缺失项，然后运行: ./run.sh task1-check"
elif (( ${#missing_figs[@]} > 0 )); then
  first_missing="${missing_figs[0]}"
  case "${first_missing}" in
    fig_6_1_*|fig_6_2_*)
      echo "- 下一步先跑静态场地并补 Gazebo 截图: ./run.sh clean && ./run.sh sim-static"
      ;;
    fig_7_*)
      echo "- 下一步补建图证据: ./run.sh mapping，另开终端 ./run.sh teleop，再运行 ./run.sh task1-runtime-check mapping"
      ;;
    fig_8_*)
      echo "- 下一步补导航证据: ./run.sh clean && ./run.sh sim-static，另开终端 ./run.sh nav，再运行 ./run.sh task1-runtime-check nav"
      ;;
    fig_9_*)
      echo "- 下一步补动态障碍物扩展示范: ./run.sh clean && ./run.sh sim-dynamic，另开终端 ./run.sh nav-3d"
      ;;
    *)
      echo "- 按 tasks/task1/TASK1_EVIDENCE_TODO.md 补齐剩余截图"
      ;;
  esac
elif [[ "${experiment_pending}" != "0" || "${todo_total}" != "0" ]]; then
  echo "- 截图已基本齐，下一步整理 EXPERIMENT_RECORD.md 和报告中的待填字段"
elif [[ ! -f "${REPORT_PDF}" ]]; then
  echo "- 下一步生成报告 PDF: ./run.sh task1-build-report"
else
  echo "- 下一步运行最终严格检查: ./run.sh task1-check --strict && ./run.sh task1-delivery-check --strict"
  echo "- 严格检查通过后创建压缩包: ./run.sh task1-package-preview --create"
fi

echo
echo "汇总: OK=${pass_count}, TODO=${warn_count}, 缺失=${fail_count}"

if (( fail_count > 0 )); then
  exit 1
fi

exit 0
