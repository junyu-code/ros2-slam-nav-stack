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
SNAPSHOT_PATH="tasks/task1/TASK1_STATUS_SNAPSHOT.md"
RUNTIME_SNAPSHOT="tasks/task1/TASK1_RUNTIME_LAST.md"
RUNTIME_HISTORY_DIR="tasks/task1/runtime_checks"
GENERATED_TRIALS_MD="tasks/task1/STATIC_TRIALS_TABLE.md"
GENERATED_TRIALS_TEX="tasks/task1/report_latex/generated_static_trials.tex"
STATIC_WORLD="src/slam_nav_simulation/world/nav_test_world/nav_test_world.world"
DYNAMIC_WORLD="src/slam_nav_simulation/world/nav_test_world/nav_test_world_dynamic.world"
MAP_YAML="src/slam_nav_bringup/map/nav_test_map.yaml"
MAP_PGM="src/slam_nav_bringup/map/nav_test_map.pgm"
WRITE_FILE=true

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-snapshot [--stdout] [--output <path>]

说明：
  生成 task1 当前证据状态快照，不启动 Gazebo、RViz、Nav2 或任何 ROS GUI。
  默认写入 tasks/task1/TASK1_STATUS_SNAPSHOT.md，方便后续接着跑实验和写报告。

可选参数：
  --stdout         只输出到终端，不写文件。
  --output <path>  指定快照写入路径。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stdout)
      WRITE_FILE=false
      ;;
    --output)
      if [[ -z "${2:-}" ]]; then
        echo "[task1-snapshot] --output 需要路径参数" >&2
        exit 2
      fi
      SNAPSHOT_PATH="$2"
      shift
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task1-snapshot] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

required_figs=(
  "fig_6_1_gazebo_world.png|Gazebo 静态场地总览|./run.sh clean && ./run.sh sim-static"
  "fig_6_2_robot_model.png|机器人模型和传感器|./run.sh sim-static"
  "fig_7_1_mapping_rviz.png|RViz 建图过程|./run.sh mapping + ./run.sh teleop"
  "fig_7_2_saved_map.png|保存后的地图|./run.sh save-map nav_test_map"
  "fig_8_1_nav2_map_loaded.png|Nav2 加载地图|./run.sh nav"
  "fig_8_2_global_path.png|全局路径|RViz Nav2 Goal"
  "fig_8_3_avoid_obstacle.png|静态避障过程|RViz 导航过程中截图"
  "fig_8_4_goal_reached.png|到达目标点|RViz 到达目标后截图"
  "fig_9_1_dynamic_obstacle.png|动态障碍物扩展示范|./run.sh sim-dynamic + ./run.sh nav-3d"
)

optional_figs=(
  "fig_8_5_backup_recovery.png|后退恢复过程"
  "fig_9_2_rgbd_visual_obstacles.png|RGB-D 近场障碍点云"
  "fig_9_3_perception_adapter.png|感知接口说明"
)

count_matches() {
  local pattern="$1"
  shift
  grep -RInE "${pattern}" "$@" 2>/dev/null | wc -l | tr -d ' '
}

file_status() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    printf '已存在，%s，更新时间 %s' \
      "$(stat -c '%s bytes' "${path}")" \
      "$(date -d "@$(stat -c '%Y' "${path}")" '+%Y-%m-%d %H:%M:%S')"
  else
    printf '缺失'
  fi
}

missing_required_figs=()
for item in "${required_figs[@]}"; do
  IFS='|' read -r file desc hint <<<"${item}"
  if [[ ! -f "${FIG_DIR}/${file}" ]]; then
    missing_required_figs+=("${file}|${desc}|${hint}")
  fi
done

todo_sources=("${EXPERIMENT_RECORD}" "${REPORT_DRAFT}" "${REPORT_TEX}")
[[ -f "${GENERATED_TRIALS_MD}" ]] && todo_sources+=("${GENERATED_TRIALS_MD}")
[[ -f "${GENERATED_TRIALS_TEX}" ]] && todo_sources+=("${GENERATED_TRIALS_TEX}")
todo_total="$(count_matches '待填|待补|待替换|【待插图|placeholderfigure' "${todo_sources[@]}")"
experiment_pending="$(grep -o '待填' "${EXPERIMENT_RECORD}" 2>/dev/null | wc -l | tr -d ' ')"

runtime_history_count=0
runtime_history_latest="缺失"
if [[ -d "${RUNTIME_HISTORY_DIR}" ]]; then
  runtime_history_count="$(find "${RUNTIME_HISTORY_DIR}" -maxdepth 1 -type f -name '*.md' | wc -l | tr -d ' ')"
  runtime_history_latest="$(find "${RUNTIME_HISTORY_DIR}" -maxdepth 1 -type f -name '*.md' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2-)"
  [[ -z "${runtime_history_latest}" ]] && runtime_history_latest="暂无"
fi

map_stale=false
map_stale_status="未发现过期"
if [[ -f "${MAP_YAML}" && -f "${MAP_PGM}" ]]; then
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
    map_stale_status="场地内容相对 Git 基线已有修改且晚于 nav_test_map，正式验收前需要重新建图保存"
  fi
else
  map_stale_status="默认地图不完整，需要先建图并保存 nav_test_map"
fi

world_check_status="未运行"
if [[ -x "${SCRIPT_DIR}/task1_world_check.sh" ]]; then
  if "${SCRIPT_DIR}/task1_world_check.sh" >/tmp/task1_snapshot_world.log 2>&1; then
    world_check_status="通过：SDF、固定障碍物、动态障碍物插件和旧场地兼容性 OK"
  else
    world_check_status="未通过：请运行 \`./run.sh task1-world-check\` 查看具体问题"
  fi
else
  world_check_status="缺少脚本：scripts/task1_world_check.sh"
fi

map_check_status="未运行"
if [[ -x "${SCRIPT_DIR}/task1_map_check.sh" ]]; then
  if "${SCRIPT_DIR}/task1_map_check.sh" >/tmp/task1_snapshot_map.log 2>&1; then
    map_check_summary="$(grep -E '^  - (resolution|origin|image size|map extent):' /tmp/task1_snapshot_map.log | sed 's/^  - //' | awk 'BEGIN{sep=""}{printf "%s%s", sep, $0; sep="; "} END{print ""}')"
    map_check_status="通过：${map_check_summary:-可运行 \`./run.sh task1-map-check\` 查看详情}"
  else
    map_check_status="未通过：请运行 \`./run.sh task1-map-check\` 查看具体问题"
  fi
else
  map_check_status="缺少脚本：scripts/task1_map_check.sh"
fi

task1_check_status="未运行"
if "${SCRIPT_DIR}/task1_preflight.sh" >/tmp/task1_snapshot_check.log 2>&1; then
  task1_check_status="普通预检通过；若还有截图/实验 warning，最终用 --strict 再确认"
else
  task1_check_status="普通预检未通过，请运行 ./run.sh task1-check 查看"
fi

experiment_check_status="未运行"
if "${SCRIPT_DIR}/task1_experiment_check.sh" >/tmp/task1_snapshot_experiment.log 2>&1; then
  if "${SCRIPT_DIR}/task1_experiment_check.sh" --strict >/tmp/task1_snapshot_experiment_strict.log 2>&1; then
    experiment_check_status="严格检查通过，静态避障实验已满足最终要求"
  else
    experiment_check_status="普通检查可执行，但严格检查未通过；通常表示 10 次记录、截图或 80% 成功率仍待补"
  fi
else
  experiment_check_status="实验记录检查脚本未通过，请运行 ./run.sh task1-experiment-check --show-rows 查看"
fi

report_audit_status="未运行"
if "${SCRIPT_DIR}/task1_report_audit.sh" >/tmp/task1_snapshot_report.log 2>&1; then
  if "${SCRIPT_DIR}/task1_report_audit.sh" --strict >/tmp/task1_snapshot_report_strict.log 2>&1; then
    report_audit_status="严格审计通过，报告材料已满足最终要求"
  else
    report_audit_status="普通审计通过，但严格审计未通过；通常表示截图、待填字段或 PDF 更新时间仍待处理"
  fi
else
  report_audit_status="报告审计未通过，请运行 ./run.sh task1-report-audit 查看"
fi

next_step="- 暂无自动建议，请运行 ./run.sh task1-status 复核。"
if [[ "${map_stale}" == "true" ]]; then
  next_step="- 当前场地内容相对 Git 基线已有修改且晚于 `nav_test_map`；下一步优先重新建图并保存地图：`./run.sh clean && ./run.sh sim-static`，另开终端运行 `./run.sh mapping` 和 `./run.sh teleop`，扫完后执行 `./run.sh save-map nav_test_map`。这一步可以顺手补图 6-1、6-2、7-1、7-2。"
elif (( ${#missing_required_figs[@]} > 0 )); then
  IFS='|' read -r first_file _first_desc first_hint <<<"${missing_required_figs[0]}"
  case "${first_file}" in
    fig_6_1_*|fig_6_2_*)
      next_step="- 下一步先补 Gazebo 静态场地截图：\`${first_hint}\`。"
      ;;
    fig_7_*)
      next_step="- 下一步补建图证据：\`./run.sh mapping\`，另开终端 \`./run.sh teleop\`，再运行 \`./run.sh task1-runtime-check mapping\`。"
      ;;
    fig_8_*)
      next_step="- 下一步补导航证据：\`./run.sh clean && ./run.sh sim-static\`，另开终端 \`./run.sh nav\`，再运行 \`./run.sh task1-runtime-check nav\`。"
      ;;
    fig_9_*)
      next_step="- 下一步补动态障碍物扩展示范：\`./run.sh clean && ./run.sh sim-dynamic\`，另开终端 \`./run.sh nav-3d\`。"
      ;;
  esac
elif [[ "${experiment_pending}" != "0" || "${todo_total}" != "0" ]]; then
  next_step="- 截图已基本齐，下一步整理 \`EXPERIMENT_RECORD.md\` 和报告里的待填字段。"
elif [[ ! -f "${REPORT_PDF}" ]]; then
  next_step="- 下一步生成报告 PDF：\`./run.sh task1-build-report\`。"
else
  next_step="- 下一步运行最终严格检查：\`./run.sh task1-finalize\`。"
fi

render_snapshot() {
  local now
  now="$(date '+%Y-%m-%d %H:%M:%S %z')"

  cat <<EOF
# Task1 当前状态快照

> 自动生成时间：${now}
>
> 生成命令：\`./run.sh task1-snapshot\`
>
> 说明：本文件用于记录当前证据缺口和下一步动作。它不代替真实实验截图，也不代替最终 strict 检查。

## 1. 基础状态

| 项目 | 状态 |
|---|---|
| 工作区 | \`${WORKSPACE_DIR}\` |
| Git 状态 | $(if git diff --quiet && git diff --cached --quiet; then echo "干净"; else echo "存在未提交改动"; fi) |
| 默认地图 yaml | $(file_status "${MAP_YAML}") |
| 默认地图 pgm | $(file_status "${MAP_PGM}") |
| 地图与场地版本 | ${map_stale_status} |
| 仿真场地 world 检查 | ${world_check_status} |
| 默认地图元数据检查 | ${map_check_status} |
| 结课报告 PDF | $(file_status "${REPORT_PDF}") |
| 运行时 latest 快照 | $(file_status "${RUNTIME_SNAPSHOT}") |
| 运行时历史快照 | ${runtime_history_count} 份，最新：${runtime_history_latest} |
| task1 普通预检 | ${task1_check_status} |
| 静态避障实验检查 | ${experiment_check_status} |
| 结课报告审计 | ${report_audit_status} |

## 2. 必需截图

| 文件 | 说明 | 状态 | 建议动作 |
|---|---|---|---|
EOF

  for item in "${required_figs[@]}"; do
    IFS='|' read -r file desc hint <<<"${item}"
    if [[ -f "${FIG_DIR}/${file}" ]]; then
      printf '| `%s` | %s | 已补 | - |\n' "${file}" "${desc}"
    else
      printf '| `%s` | %s | 缺失 | `%s` |\n' "${file}" "${desc}" "${hint}"
    fi
  done

  cat <<EOF

补图辅助：

~~~bash
./run.sh task1-figures
./run.sh task1-figures path 6-1
./run.sh task1-figures import 6-1 <源 PNG>
~~~

\`task1-figures\` 只复制你已经截好的 PNG，不生成或伪造实验截图。

## 3. 可选扩展截图

| 文件 | 说明 | 状态 |
|---|---|---|
EOF

  for item in "${optional_figs[@]}"; do
    IFS='|' read -r file desc <<<"${item}"
    if [[ -f "${FIG_DIR}/${file}" ]]; then
      printf '| `%s` | %s | 已补 |\n' "${file}" "${desc}"
    else
      printf '| `%s` | %s | 可选未补 |\n' "${file}" "${desc}"
    fi
  done

  cat <<EOF

## 4. 实验与报告待填

| 项目 | 数量/状态 |
|---|---|
| 实验记录和报告占位总数 | ${todo_total} |
| \`EXPERIMENT_RECORD.md\` 中“待填”数量 | ${experiment_pending} |
| 普通实验检查 | ${experiment_check_status} |
| 普通报告审计 | ${report_audit_status} |

## 5. 下一步建议

${next_step}

建议每次完成一批截图或实验记录后重新运行：

\`\`\`bash
cd ~/slam_nav_ws
./run.sh task1-snapshot
./run.sh task1-status
\`\`\`

最终提交前再运行：

\`\`\`bash
cd ~/slam_nav_ws
./run.sh task1-finalize
\`\`\`
EOF
}

if [[ "${WRITE_FILE}" == "true" ]]; then
  mkdir -p "$(dirname "${SNAPSHOT_PATH}")"
  render_snapshot > "${SNAPSHOT_PATH}"
  echo "[task1-snapshot] 已写入: ${SNAPSHOT_PATH}"
else
  render_snapshot
fi
