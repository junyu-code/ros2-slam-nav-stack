#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

STRICT=false
if [[ "${1:-}" == "--strict" ]]; then
  STRICT=true
fi

errors=0
warnings=0

ok() {
  echo "[task1-delivery] OK: $*"
}

warn() {
  warnings=$((warnings + 1))
  echo "[task1-delivery] WARN: $*" >&2
}

fail() {
  errors=$((errors + 1))
  echo "[task1-delivery] FAIL: $*" >&2
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

check_optional_file() {
  local path="$1"
  local desc="$2"
  if [[ -f "${path}" ]]; then
    ok "${desc}: ${path}"
  else
    warn "待补 ${desc}: ${path}"
  fi
}

echo "[task1-delivery] 工作区: ${WORKSPACE_DIR}"
echo "[task1-delivery] 建议压缩包名: 3232072072234+佘俊谕.zip"

check_file "run.sh" "根目录统一入口"
check_dir "scripts" "脚本目录"
check_dir "src" "源码目录"
check_dir "tasks/task1" "task1 材料目录"
check_file "README.md" "项目 README"
check_file "PROJECT_PROCESS.md" "项目过程记录"

check_file "tasks/task1/STUDENT_INFO.md" "个人信息记录"
check_file "tasks/task1/DELIVERY_CHECKLIST.md" "交付检查清单"
check_file "tasks/task1/TASK1_FINAL_RUNBOOK.md" "task1 最终 Runbook"
check_file "tasks/task1/TASK1_EVIDENCE_TODO.md" "task1 剩余证据采集清单"
check_file "tasks/task1/RUN_AND_SCREENSHOT_STEPS.md" "运行与截图步骤"
check_file "tasks/task1/EXPERIMENT_RECORD.md" "实验记录表"
check_file "tasks/task1/SLAM_FINAL_REPORT_DRAFT.md" "Markdown 报告草稿"
check_file "tasks/task1/report_latex/main.tex" "LaTeX 结课报告源文件"
check_file "tasks/task1/homework_latex/main.tex" "LaTeX 平时作业源文件"

check_optional_file "tasks/task1/report_latex/main.pdf" "结课报告 PDF"
check_optional_file "tasks/task1/homework_latex/main.pdf" "平时作业 PDF"

check_file "src/slam_nav_bringup/map/nav_test_map.yaml" "默认导航地图 yaml"
check_file "src/slam_nav_bringup/map/nav_test_map.pgm" "默认导航地图 pgm"

FIG_DIR="tasks/task1/report_latex/figures"
check_dir "${FIG_DIR}" "报告截图目录"

required_figs=(
  "fig_6_1_gazebo_world.png|图 6-1 Gazebo 静态场地总览"
  "fig_6_2_robot_model.png|图 6-2 机器人模型和传感器"
  "fig_7_1_mapping_rviz.png|图 7-1 RViz 建图过程"
  "fig_7_2_saved_map.png|图 7-2 保存后的地图"
  "fig_8_1_nav2_map_loaded.png|图 8-1 Nav2 加载地图"
  "fig_8_2_global_path.png|图 8-2 全局路径"
  "fig_8_3_avoid_obstacle.png|图 8-3 静态避障过程"
  "fig_8_4_goal_reached.png|图 8-4 到达目标点"
  "fig_9_1_dynamic_obstacle.png|图 9-1 动态障碍物扩展示范"
)

for item in "${required_figs[@]}"; do
  IFS='|' read -r file desc <<<"${item}"
  check_optional_file "${FIG_DIR}/${file}" "${desc}"
done

optional_figs=(
  "fig_8_5_backup_recovery.png|图 8-5 后退恢复过程"
  "fig_9_2_rgbd_visual_obstacles.png|图 9-2 RGB-D 近场障碍点云"
  "fig_9_3_perception_adapter.png|图 9-3 感知接口说明"
)

for item in "${optional_figs[@]}"; do
  IFS='|' read -r file desc <<<"${item}"
  if [[ -f "${FIG_DIR}/${file}" ]]; then
    ok "可选截图已补: ${desc}: ${FIG_DIR}/${file}"
  else
    echo "[task1-delivery] NOTE: 可选截图未补: ${desc}: ${FIG_DIR}/${file}"
  fi
done

todo_count="$(grep -RInE '待填|待补|待替换|【待插图|placeholderfigure' \
  tasks/task1/EXPERIMENT_RECORD.md \
  tasks/task1/SLAM_FINAL_REPORT_DRAFT.md \
  tasks/task1/report_latex/main.tex 2>/dev/null | wc -l | tr -d ' ')"
if [[ "${todo_count}" == "0" ]]; then
  ok "实验记录和报告没有待填/待替换占位"
else
  warn "实验记录或报告仍有 ${todo_count} 处待填/待替换占位"
fi

experiment_pending="$(grep -o '待填' tasks/task1/EXPERIMENT_RECORD.md 2>/dev/null | wc -l | tr -d ' ')"
if [[ "${experiment_pending}" == "0" ]]; then
  ok "实验记录表中未发现待填字段"
else
  warn "实验记录表仍有 ${experiment_pending} 个待填字段；10 次静态避障实验完成后再清理"
fi

if [[ -x "scripts/task1_experiment_check.sh" ]]; then
  experiment_args=()
  if [[ "${STRICT}" == "true" ]]; then
    experiment_args+=(--strict)
  fi
  if "scripts/task1_experiment_check.sh" "${experiment_args[@]}"; then
    ok "静态避障实验记录检查已执行"
  else
    warn "静态避障实验记录检查未通过；请先补齐 10 次记录并保证成功率 >= 80%"
  fi
else
  fail "缺少静态避障实验记录检查脚本: scripts/task1_experiment_check.sh"
fi

competition_hits="$(grep -RInE 'RoboMaster|RMUC|哨兵|云台|比赛模式|比赛业务' \
  tasks/task1/SLAM_FINAL_REPORT_DRAFT.md \
  tasks/task1/report_latex/main.tex 2>/dev/null || true)"
if [[ -z "${competition_hits}" ]]; then
  ok "报告正文未发现明显无关比赛业务字段"
else
  warn "报告正文可能仍有无关比赛业务字段:"
  echo "${competition_hits}" >&2
fi

tracked_heavy="$(git ls-files | grep -E '^(build|install|log)/|(^|/)(.*\.(bag|db3|mcap|pcd|ply|las|laz|onnx|pt|pth|ckpt|engine|safetensors))$' || true)"
if [[ -z "${tracked_heavy}" ]]; then
  ok "Git 未跟踪 build/install/log/rosbag/点云/模型权重等重型产物"
else
  fail "Git 中发现不应打包进源码仓库的重型产物:"
  echo "${tracked_heavy}" >&2
fi

if [[ -n "$(git status --short)" ]]; then
  warn "当前工作区存在未提交改动；最终打包前建议先提交或确认这些改动是否需要保留"
else
  ok "当前 Git 工作区干净"
fi

echo "[task1-delivery] 建议最终压缩内容:"
cat <<'EOF'
  run.sh
  scripts/
  src/
  README.md
  PROJECT_PROCESS.md
  tasks/task1/
EOF

echo "[task1-delivery] 建议排除内容:"
cat <<'EOF'
  build/
  install/
  log/
  .git/
  .vscode/
  datasets/
  *.bag
  *.db3
  *.mcap
EOF

echo "[task1-delivery] 结构错误: ${errors}, 交付提醒: ${warnings}"

if [[ "${errors}" -ne 0 ]]; then
  exit 1
fi

if [[ "${STRICT}" == "true" && "${warnings}" -ne 0 ]]; then
  echo "[task1-delivery] --strict 模式下 warning 也视为未完成。" >&2
  exit 1
fi

exit 0
