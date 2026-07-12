#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

STRICT=false
if [[ "${1:-}" == "--strict" ]]; then
  STRICT=true
  shift
fi
if [[ $# -gt 0 ]]; then
  echo "用法：./run.sh task1-delivery-check [--strict]" >&2
  exit 2
fi

source "${SCRIPT_DIR}/task1_state.sh"

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

echo "[task1-delivery] 工作区: ${WORKSPACE_DIR}"
echo "[task1-delivery] 建议压缩包名: 3232072072234+佘俊谕.zip"

if task1_load_state; then
  task1_print_state | sed 's/^/[task1-delivery]   /'
  while IFS= read -r issue; do
    [[ -n "${issue}" ]] && warn "${issue}"
  done < <(task1_state_issues)
else
  fail "Task1 状态文件无效"
fi

check_file "run.sh" "统一入口"
check_file "README.md" "工作空间 README"
check_file "tasks/task1/README.md" "Task1 README"
check_file "tasks/task1/task1.env" "Task1 状态源"
check_file "tasks/task1/EXPERIMENT_RECORD.md" "实验记录"
check_file "tasks/task1/homework_latex/main.tex" "平时作业 LaTeX"
check_file "tasks/task1/report_latex/main.tex" "结课报告 LaTeX"
check_file "tasks/task1/evidence_media/MANIFEST.tsv" "媒体清单"
check_file "tasks/task1/evidence_media/SHA256SUMS" "媒体校验和"
check_file "tasks/task1/evidence_media/task1_rgbd_dynamic_demo_720p.mp4" "阶段性演示视频"
check_file "src/slam_nav_bringup/map/nav_test_map.yaml" "默认地图 YAML"
check_file "src/slam_nav_bringup/map/nav_test_map.pgm" "默认地图 PGM"

for pdf in tasks/task1/homework_latex/main.pdf tasks/task1/report_latex/main.pdf; do
  if [[ -f "${pdf}" ]]; then
    ok "本地 PDF 存在: ${pdf}"
  else
    warn "本地 PDF 尚未生成: ${pdf}"
  fi
done

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
for file in "${required_figs[@]}"; do
  if [[ -f "${FIG_DIR}/${file}" ]]; then
    ok "阶段性截图存在: ${file}"
  else
    warn "阶段性截图缺失: ${FIG_DIR}/${file}"
  fi
done

experiment_args=()
report_args=()
if [[ "${STRICT}" == "true" ]]; then
  experiment_args+=(--strict)
  report_args+=(--strict)
fi

if "${SCRIPT_DIR}/task1_experiment_check.sh" "${experiment_args[@]}" >/tmp/task1_delivery_experiment.log 2>&1; then
  ok "实验记录检查已执行"
else
  warn "实验记录尚未满足当前检查级别"
fi

if "${SCRIPT_DIR}/task1_report_audit.sh" "${report_args[@]}" >/tmp/task1_delivery_report.log 2>&1; then
  ok "报告审计已执行"
else
  warn "结课报告尚未满足当前检查级别"
fi

tracked_heavy="$(git ls-files | grep -E '^(build|install|log|artifacts)/|(^|/)(.*\.(bag|db3|mcap|pcd|ply|las|laz|onnx|pt|pth|ckpt|engine|safetensors))$' || true)"
if [[ -z "${tracked_heavy}" ]]; then
  ok "Git 未跟踪构建目录、点云、rosbag 或模型权重"
else
  fail "Git 中发现不应进入源码包的重型产物:"
  echo "${tracked_heavy}" >&2
fi

if [[ -n "$(git status --short)" ]]; then
  warn "Git 工作区存在未提交改动"
else
  ok "Git 工作区干净"
fi

echo "[task1-delivery] 建议最终压缩内容:"
cat <<'EOF'
  run.sh
  scripts/
  src/
  README.md
  tasks/task1/README.md
  tasks/task1/task1.env
  tasks/task1/EXPERIMENT_RECORD.md
  tasks/task1/homework_latex/
  tasks/task1/report_latex/
  tasks/task1/evidence_media/
EOF

echo "[task1-delivery] 结构错误: ${errors}, 交付提醒: ${warnings}"
if (( errors > 0 )); then
  exit 1
fi
if [[ "${STRICT}" == "true" && "${warnings}" -gt 0 ]]; then
  echo "[task1-delivery] --strict 模式下 warning 也视为未完成。" >&2
  exit 1
fi
