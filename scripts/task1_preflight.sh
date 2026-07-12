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
  echo "用法：./run.sh task1-check [--strict]" >&2
  exit 2
fi

source "${SCRIPT_DIR}/task1_state.sh"

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
  if [[ -x "${path}" ]]; then
    ok "脚本可执行: ${path}"
  else
    fail "脚本缺少或不可执行: ${path}"
  fi
}

echo "[task1-check] 工作区: ${WORKSPACE_DIR}"

if task1_load_state; then
  ok "Task1 状态文件可读取"
  task1_print_state | sed 's/^/[task1-check]   /'
  while IFS= read -r issue; do
    [[ -n "${issue}" ]] && warn "${issue}"
  done < <(task1_state_issues)
else
  fail "Task1 状态文件无效"
fi

check_executable "run.sh"
required_scripts=(
  build.sh clean.sh task1_state.sh task1_guide.sh task1_status.sh task1_snapshot.sh
  task1_world_check.sh task1_map_check.sh task1_runtime_check.sh
  task1_experiment_check.sh task1_figures.sh task1_sync_report.sh
  task1_report_audit.sh task1_delivery_check.sh task1_package_preview.sh
  build_task1_report.sh task1_finalize.sh
)
for script in "${required_scripts[@]}"; do
  check_executable "scripts/${script}"
done

# 检查 run.sh 中映射到脚本的入口，避免帮助信息和实际文件脱节。
declare -A checked_scripts=()
while IFS= read -r mapped_script; do
  [[ -n "${mapped_script}" ]] || continue
  [[ "${mapped_script}" == "__help__" ]] && continue
  if [[ -z "${checked_scripts[${mapped_script}]:-}" ]]; then
    checked_scripts["${mapped_script}"]=1
    check_executable "scripts/${mapped_script}"
  fi
done < <(
  awk '
    /\) echo "[^"]+\.sh" ;;/ {
      line = $0
      sub(/.*echo "/, "", line)
      sub(/" ;;.*/, "", line)
      print line
    }
  ' run.sh
)

check_file "README.md" "工作空间 README"
check_file "tasks/task1/README.md" "Task1 README"
check_file "tasks/task1/task1.env" "Task1 状态源"
check_file "tasks/task1/EXPERIMENT_RECORD.md" "实验事实来源"
check_file "tasks/task1/homework_latex/main.tex" "平时作业 LaTeX"
check_file "tasks/task1/report_latex/main.tex" "结课报告 LaTeX"
check_file "tasks/task1/evidence_media/MANIFEST.tsv" "媒体清单"
check_file "tasks/task1/evidence_media/SHA256SUMS" "媒体校验和"
check_file "tasks/task1/evidence_media/task1_rgbd_dynamic_demo_720p.mp4" "阶段性演示视频"

check_dir "src/slam_nav_simulation" "仿真包"
check_dir "src/slam_nav_bringup" "导航包"
check_dir "src/FAST_LIO" "FAST-LIO"
check_dir "src/pointcloud_to_laserscan" "点云投影包"
check_file "src/slam_nav_bringup/map/nav_test_map.yaml" "默认地图 YAML"
check_file "src/slam_nav_bringup/map/nav_test_map.pgm" "默认地图 PGM"
check_file "src/slam_nav_simulation/world/nav_test_world/nav_test_world.world" "静态仿真场地"

if "${SCRIPT_DIR}/task1_world_check.sh" >/tmp/task1_preflight_world.log 2>&1; then
  ok "仿真场地检查通过"
else
  fail "仿真场地检查未通过，运行 ./run.sh task1-world-check 查看"
fi

if "${SCRIPT_DIR}/task1_map_check.sh" >/tmp/task1_preflight_map.log 2>&1; then
  ok "默认地图元数据检查通过"
else
  fail "默认地图元数据检查未通过，运行 ./run.sh task1-map-check 查看"
fi

allowed_docs=(
  README.md
  tasks/task1/README.md
  tasks/task1/EXPERIMENT_RECORD.md
  tasks/task2/README.md
  tasks/task2/REAL_ROBOT_RUNBOOK.md
)
mapfile -t actual_docs < <(find . -path './.git' -prune -o \( -path './src' -o -path './ui' -o -path './Livox-SDK2' \) -prune -o -type f -name '*.md' -printf '%P\n' | sort)
mapfile -t expected_docs < <(printf '%s\n' "${allowed_docs[@]}" | sort)
if diff -u <(printf '%s\n' "${expected_docs[@]}") <(printf '%s\n' "${actual_docs[@]}") >/tmp/task1_preflight_docs.diff; then
  ok "根目录和 tasks/ 的人工 Markdown 已收敛为 5 份"
else
  fail "根目录和 tasks/ 的 Markdown 不符合约定；差异如下"
  cat /tmp/task1_preflight_docs.diff >&2
fi

todo_sources=(
  tasks/task1/EXPERIMENT_RECORD.md
  tasks/task1/homework_latex/main.tex
  tasks/task1/report_latex/main.tex
)
todo_count="$({ grep -RInE '待填|待补|待替换|【待插图' "${todo_sources[@]}" 2>/dev/null || true; } | wc -l | tr -d ' ')"
if [[ "${todo_count}" == "0" ]]; then
  ok "两份正文和实验记录没有待填占位"
else
  warn "两份正文或实验记录仍有 ${todo_count} 处待填占位"
fi

if "${SCRIPT_DIR}/task1_experiment_check.sh" >/tmp/task1_preflight_experiment.log 2>&1; then
  ok "实验记录结构可解析"
else
  fail "实验记录检查失败"
fi

for pdf in tasks/task1/homework_latex/main.pdf tasks/task1/report_latex/main.pdf; do
  if [[ -f "${pdf}" ]]; then
    ok "本地 PDF 存在: ${pdf}"
  else
    warn "本地 PDF 尚未生成: ${pdf}"
  fi
done

echo "[task1-check] 结构错误: ${errors}, 提交前提醒: ${warnings}"

if (( errors > 0 )); then
  exit 1
fi
if [[ "${STRICT}" == "true" && "${warnings}" -gt 0 ]]; then
  echo "[task1-check] --strict 模式下 warning 也视为未完成。" >&2
  exit 1
fi
