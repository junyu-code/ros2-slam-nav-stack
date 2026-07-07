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
RUNTIME_HISTORY_DIR="tasks/task1/runtime_checks"
GENERATED_TRIALS_MD="tasks/task1/STATIC_TRIALS_TABLE.md"
GENERATED_TRIALS_TEX="tasks/task1/report_latex/generated_static_trials.tex"
STATIC_WORLD="src/slam_nav_simulation/world/nav_test_world/nav_test_world.world"
DYNAMIC_WORLD="src/slam_nav_simulation/world/nav_test_world/nav_test_world_dynamic.world"
MAP_YAML="src/slam_nav_bringup/map/nav_test_map.yaml"
MAP_PGM="src/slam_nav_bringup/map/nav_test_map.pgm"
map_stale=false

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

if [[ -f "${MAP_YAML}" && -f "${MAP_PGM}" ]]; then
  pass "默认导航地图存在: nav_test_map.yaml ($(file_size "${MAP_YAML}")), nav_test_map.pgm ($(file_size "${MAP_PGM}"))"
  if [[ -x "scripts/task1_map_check.sh" ]]; then
    if "scripts/task1_map_check.sh" >/tmp/task1_status_map_check.log 2>&1; then
      map_summary="$(grep -E '^  - (resolution|origin|image size|map extent):' /tmp/task1_status_map_check.log | sed 's/^  - //' | awk 'BEGIN{sep=""}{printf "%s%s", sep, $0; sep="; "} END{print ""}')"
      pass "默认地图元数据检查通过: ${map_summary:-可运行 ./run.sh task1-map-check 查看详情}"
    else
      fail "默认地图元数据检查未通过；运行 ./run.sh task1-map-check 查看具体问题"
    fi
  else
    fail "缺少默认地图元数据检查脚本: scripts/task1_map_check.sh"
  fi
  # 只有 world 内容确实偏离 Git 基线时，才提示默认地图需要重建；单纯 touch 不再误报。
  map_world_mismatch=false
  for world_file in "${STATIC_WORLD}"; do
    if [[ "${world_file}" -nt "${MAP_YAML}" || "${world_file}" -nt "${MAP_PGM}" ]]; then
      if ! git diff --quiet -- "${world_file}" 2>/dev/null; then
        map_world_mismatch=true
      fi
    fi
  done
  if [[ "${map_world_mismatch}" == "true" ]]; then
    map_stale=true
    warn "仿真场地内容相对 Git 基线已有修改且晚于默认地图；正式验收前建议重新执行 ./run.sh sim-static、./run.sh mapping、./run.sh save-map nav_test_map"
  fi
else
  fail "默认导航地图不完整，先完成建图并执行 ./run.sh save-map nav_test_map"
fi

if [[ -x "scripts/task1_world_check.sh" ]]; then
  if "scripts/task1_world_check.sh" >/tmp/task1_status_world_check.log 2>&1; then
    pass "仿真场地 world 检查通过: SDF、固定障碍物、动态障碍物插件和旧场地兼容性 OK"
  else
    fail "仿真场地 world 检查未通过；运行 ./run.sh task1-world-check 查看具体问题"
  fi
else
  fail "缺少仿真场地检查脚本: scripts/task1_world_check.sh"
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

if [[ -d "${RUNTIME_HISTORY_DIR}" ]]; then
  runtime_history_count="$(find "${RUNTIME_HISTORY_DIR}" -maxdepth 1 -type f -name '*.md' | wc -l | tr -d ' ')"
  latest_runtime_history="$(find "${RUNTIME_HISTORY_DIR}" -maxdepth 1 -type f -name '*.md' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
  if [[ "${runtime_history_count}" != "0" ]]; then
    pass "运行时检查历史快照 ${runtime_history_count} 份，最新: ${latest_runtime_history}"
  else
    warn "运行时检查历史目录存在但还没有快照: ${RUNTIME_HISTORY_DIR}"
  fi
else
  warn "运行时检查历史目录尚未生成；使用 --save 后会自动创建 ${RUNTIME_HISTORY_DIR}"
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
todo_sources=("${EXPERIMENT_RECORD}" "${REPORT_DRAFT}" "${REPORT_TEX}")
[[ -f "${GENERATED_TRIALS_MD}" ]] && todo_sources+=("${GENERATED_TRIALS_MD}")
[[ -f "${GENERATED_TRIALS_TEX}" ]] && todo_sources+=("${GENERATED_TRIALS_TEX}")
todo_total="$(count_matches '待填|待补|待替换|【待插图|placeholderfigure' "${todo_sources[@]}")"
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
elif [[ "${map_stale}" == "true" ]]; then
  echo "- 当前场地内容相对 Git 基线已有修改且晚于 nav_test_map；下一步优先重新建图并保存地图: ./run.sh clean && ./run.sh sim-static，另开终端 ./run.sh mapping 和 ./run.sh teleop，扫完后执行 ./run.sh save-map nav_test_map"
  echo "- 这一步可以顺手补图 6-1、6-2、7-1、7-2；新地图保存后再继续 ./run.sh nav 做 8 组导航截图和 10 次静态避障。"
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

if (( ${#missing_figs[@]} > 0 )); then
  echo "- 截图保存到临时目录后，可用 ./run.sh task1-figures 查看标准文件名，并用 ./run.sh task1-figures import <图号> <源 PNG> 导入。"
fi

echo
echo "汇总: OK=${pass_count}, TODO=${warn_count}, 缺失=${fail_count}"

if (( fail_count > 0 )); then
  exit 1
fi

exit 0
