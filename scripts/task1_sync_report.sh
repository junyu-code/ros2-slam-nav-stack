#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

RECORD_FILE="tasks/task1/EXPERIMENT_RECORD.md"
GENERATED_TEX="tasks/task1/report_latex/generated_static_trials.tex"
QUIET=false

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-sync-report [--quiet]

说明：
  从 EXPERIMENT_RECORD.md 读取静态避障表，只生成报告引用的 LaTeX 片段。
  实验事实始终只在 EXPERIMENT_RECORD.md 中人工维护。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --quiet)
      QUIET=true
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task1-sync] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

log() {
  [[ "${QUIET}" == "true" ]] || echo "[task1-sync] $*"
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "${value}"
}

normalize_cell() {
  local value
  value="$(trim "$1")"
  [[ -n "${value}" ]] || value="待填"
  printf '%s' "${value}"
}

latex_escape() {
  printf '%s' "$1" | sed -e 's/\\/\\textbackslash{}/g' -e 's/[&%$#_]/\\&/g' -e 's/{/\\{/g' -e 's/}/\\}/g'
}

is_positive() {
  local value="$1"
  [[ "${value}" == *"是"* || "${value}" == *"成功"* || "${value}" == *"通过"* ||
     "${value}" == *"yes"* || "${value}" == *"Yes"* || "${value}" == "1" ]]
}

if [[ ! -f "${RECORD_FILE}" ]]; then
  echo "[task1-sync] 缺少实验记录: ${RECORD_FILE}" >&2
  exit 1
fi

mkdir -p "$(dirname "${GENERATED_TEX}")"
tmp_tex="$(mktemp)"
trap 'rm -f "${tmp_tex}"' EXIT

cat >"${tmp_tex}" <<'EOF'
% 本文件由 ./run.sh task1-sync-report 自动生成。
% 数据来源：tasks/task1/EXPERIMENT_RECORD.md
% 请勿直接修改本文件。
EOF

in_table=false
rows=0
success_count=0

while IFS= read -r line; do
  if [[ "${line}" == *"| 编号 | 起点 | 目标点 |"* && "${line}" == *"是否成功"* ]]; then
    in_table=true
    continue
  fi
  [[ "${in_table}" == "true" ]] || continue
  if [[ "${line}" != "|"* ]]; then
    (( rows == 0 )) || break
    continue
  fi
  [[ "${line}" == *"---"* ]] && continue

  row="${line#|}"
  row="${row%|}"
  IFS='|' read -r c_num c_start c_goal c_obstacle c_arrived c_collision c_success c_shot c_note c_extra <<<"${row}"

  num="$(trim "${c_num:-}")"
  [[ "${num}" =~ ^[0-9]+$ ]] || continue

  start="$(normalize_cell "${c_start:-}")"
  goal="$(normalize_cell "${c_goal:-}")"
  obstacle="$(normalize_cell "${c_obstacle:-}")"
  arrived="$(normalize_cell "${c_arrived:-}")"
  collision="$(normalize_cell "${c_collision:-}")"
  success="$(normalize_cell "${c_success:-}")"

  rows=$((rows + 1))
  if is_positive "${success}" && ! is_positive "${collision}"; then
    success_count=$((success_count + 1))
  fi

  cells=(
    "$(latex_escape "${num}")"
    "$(latex_escape "${start}")"
    "$(latex_escape "${goal}")"
    "$(latex_escape "${obstacle}")"
    "$(latex_escape "${arrived}")"
    "$(latex_escape "${collision}")"
    "$(latex_escape "${success}")"
  )
  printf '  %s & %s & %s & %s & %s & %s & %s \\\\\n' "${cells[@]}" >>"${tmp_tex}"
done <"${RECORD_FILE}"

if (( rows == 0 )); then
  echo "[task1-sync] 未解析到静态避障实验表" >&2
  exit 1
fi

printf '  \\bottomrule\n' >>"${tmp_tex}"
if [[ -f "${GENERATED_TEX}" ]] && cmp -s "${tmp_tex}" "${GENERATED_TEX}"; then
  log "内容未变化: ${GENERATED_TEX}"
else
  mv "${tmp_tex}" "${GENERATED_TEX}"
  log "已更新: ${GENERATED_TEX}"
fi
trap - EXIT

log "解析到 ${rows} 行，当前阶段性成功 ${success_count} 次。"
if (( rows < 10 )); then
  echo "[task1-sync] WARN: 静态避障实验少于 10 行。" >&2
fi
