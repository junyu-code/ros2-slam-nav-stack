#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

if [[ "${1:-}" == "--brief" ]]; then
  tmp_output="$(mktemp)"
  trap 'rm -f "${tmp_output}"' EXIT
  set +e
  "$0" >"${tmp_output}" 2>&1
  status=$?
  set -e
  awk '/^Task1 当前状态/ || /^  TASK1_/ || /^\[TODO\]/ || /^\[缺失\]/ || /^下一步:/ || /^汇总:/' "${tmp_output}"
  exit "${status}"
fi

if [[ $# -gt 0 ]]; then
  echo "用法：./run.sh task1-status [--brief]" >&2
  exit 2
fi

source "${SCRIPT_DIR}/task1_state.sh"

ok_count=0
todo_count=0
fail_count=0

ok() {
  ok_count=$((ok_count + 1))
  echo "[OK] $*"
}

todo() {
  todo_count=$((todo_count + 1))
  echo "[TODO] $*"
}

fail() {
  fail_count=$((fail_count + 1))
  echo "[缺失] $*"
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

echo "Task1 当前状态"
echo "工作区: ${WORKSPACE_DIR}"

if task1_load_state; then
  task1_print_state | sed 's/^/  /'
else
  fail "task1.env 无法读取"
fi

while IFS= read -r issue; do
  [[ -n "${issue}" ]] && todo "${issue}"
done < <(task1_state_issues)

echo
echo "1. 权威来源"
check_file "tasks/task1/README.md" "Task1 说明"
check_file "tasks/task1/task1.env" "Task1 状态源"
check_file "tasks/task1/EXPERIMENT_RECORD.md" "实验事实来源"
check_file "tasks/task1/homework_latex/main.tex" "平时作业正文"
check_file "tasks/task1/report_latex/main.tex" "结课报告正文"
check_file "tasks/task1/evidence_media/MANIFEST.tsv" "媒体清单"
check_file "tasks/task1/evidence_media/SHA256SUMS" "媒体校验和"

echo
echo "2. 地图与阶段性证据"
check_file "src/slam_nav_bringup/map/nav_test_map.yaml" "默认地图 YAML"
check_file "src/slam_nav_bringup/map/nav_test_map.pgm" "默认地图 PGM"

FIG_DIR="tasks/task1/report_latex/figures"
required_figs=(
  fig_6_1_gazebo_world.png
  fig_7_1_mapping_rviz.png
  fig_7_2_saved_map.png
  fig_8_1_nav2_map_loaded.png
  fig_8_2_global_path.png
  fig_8_3_avoid_obstacle.png
  fig_8_4_goal_reached.png
  fig_9_1_dynamic_obstacle.png
)
present_figs=0
for file in "${required_figs[@]}"; do
  [[ -f "${FIG_DIR}/${file}" ]] && present_figs=$((present_figs + 1))
done
if (( present_figs == ${#required_figs[@]} )); then
  ok "阶段性必需截图齐全: ${present_figs}/${#required_figs[@]}"
else
  todo "阶段性必需截图: ${present_figs}/${#required_figs[@]}"
fi

if [[ -f "tasks/task1/evidence_media/task1_rgbd_dynamic_demo_720p.mp4" ]]; then
  ok "阶段性演示视频存在"
else
  todo "阶段性演示视频未归档"
fi

echo
echo "3. 实验与报告"
tmp_experiment="$(mktemp)"
trap 'rm -f "${tmp_experiment}"' EXIT
if "${SCRIPT_DIR}/task1_experiment_check.sh" >"${tmp_experiment}" 2>&1; then
  summary="$(grep '^\[task1-experiment\] 统计:' "${tmp_experiment}" | tail -n 1 || true)"
  ok "实验记录可解析${summary:+；${summary#*: }}"
else
  fail "实验记录结构无法解析，运行 ./run.sh task1-experiment-check 查看"
fi

for pdf in tasks/task1/homework_latex/main.pdf tasks/task1/report_latex/main.pdf; do
  if [[ -f "${pdf}" ]]; then
    ok "本地 PDF 存在: ${pdf}"
  else
    todo "本地 PDF 尚未生成: ${pdf}"
  fi
done

if [[ -f "artifacts/task1/status.txt" ]]; then
  ok "本地状态快照存在: artifacts/task1/status.txt"
else
  todo "本地状态快照尚未生成，可运行 ./run.sh task1-snapshot"
fi

echo
echo "4. Git"
if [[ -z "$(git status --short)" ]]; then
  ok "Git 工作区干净"
else
  todo "Git 工作区存在未提交改动"
fi

echo
echo "下一步:"
if (( fail_count > 0 )); then
  echo "- 先修复缺失的权威文件或实验表结构。"
elif [[ "${TASK1_NAVIGATION_SCHEME}" == "pending" ]]; then
  echo "- 按 tasks/task1/README.md 的统一条件比较 nav、nav-3d、nav-full，并冻结一个主方案。"
elif [[ "${TASK1_EVIDENCE_STATE}" != "final" ]]; then
  echo "- 使用已冻结方案重新完成正式 10 次实验，再把证据状态改为 final。"
elif [[ "${TASK1_STATE}" != "ready" ]]; then
  echo "- 完成两份 LaTeX 正文和普通检查，再把 Task1 状态改为 ready。"
else
  echo "- 运行 strict 检查和最终打包预览。"
fi

echo
echo "汇总: OK=${ok_count}, TODO=${todo_count}, 缺失=${fail_count}"

if (( fail_count > 0 )); then
  exit 1
fi
