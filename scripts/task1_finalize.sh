#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

CREATE=false
ALLOW_WARNINGS=false
SKIP_REPORT_BUILD=false
REPORT_ONCE=false
KEEP_GOING=false
LIST_FILES=false

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-finalize [--create] [--allow-warnings] [--skip-report-build] [--report-once] [--keep-going] [--list]

说明：
  task1 最终交付编排入口，不启动 Gazebo、RViz 或 Nav2。
  默认流程：查看 task1 状态 -> 保存本地文本快照 -> 同步实验表到报告 -> 检查静态避障实验记录 -> 编译结课报告 PDF -> 报告审计 -> strict 预检 -> strict 交付检查 -> 预览压缩包。
  加 --create 后才会真正创建 zip。

可选参数：
  --create             严格检查通过后创建 dist/3232072072234+佘俊谕.zip。
  --allow-warnings     草稿模式：warning 不阻止流程，创建 zip 时传给 task1-package-preview。
  --skip-report-build  跳过 LaTeX 报告编译。
  --report-once        报告只编译 1 遍，适合快速检查。
  --keep-going         传给 task1-build-report，普通 LaTeX warning 时继续编译。
  --list               预览或创建前列出将进入压缩包的文件。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --create)
      CREATE=true
      ;;
    --allow-warnings)
      ALLOW_WARNINGS=true
      ;;
    --skip-report-build)
      SKIP_REPORT_BUILD=true
      ;;
    --report-once)
      REPORT_ONCE=true
      ;;
    --keep-going)
      KEEP_GOING=true
      ;;
    --list)
      LIST_FILES=true
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task1-finalize] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

step() {
  echo
  echo "[task1-finalize] $*"
}

run_check() {
  local script="$1"
  shift
  "${SCRIPT_DIR}/${script}" "$@"
}

step "1/9 查看当前 task1 状态"
run_check task1_status.sh

step "2/9 保存 task1 本地文本快照"
run_check task1_snapshot.sh

step "3/9 同步静态避障实验表到报告片段"
run_check task1_sync_report.sh

experiment_args=(--show-rows --next)
if [[ "${ALLOW_WARNINGS}" != "true" ]]; then
  experiment_args+=(--strict)
fi

step "4/9 检查静态避障实验记录和成功率"
run_check task1_experiment_check.sh "${experiment_args[@]}"

if [[ "${SKIP_REPORT_BUILD}" == "true" ]]; then
  step "5/9 跳过报告 PDF 编译"
else
  report_args=()
  if [[ "${REPORT_ONCE}" == "true" ]]; then
    report_args+=(--once)
  fi
  if [[ "${KEEP_GOING}" == "true" ]]; then
    report_args+=(--keep-going)
  fi
  step "5/9 编译 task1 结课报告 PDF"
  run_check build_task1_report.sh "${report_args[@]}"
fi

preflight_args=()
delivery_args=()
report_audit_args=()
if [[ "${ALLOW_WARNINGS}" != "true" ]]; then
  preflight_args+=(--strict)
  delivery_args+=(--strict)
  report_audit_args+=(--strict)
fi

step "6/9 审计 task1 结课报告源文件、截图和 PDF"
run_check task1_report_audit.sh "${report_audit_args[@]}"

step "7/9 运行 task1 结构预检"
run_check task1_preflight.sh "${preflight_args[@]}"

step "8/9 运行 task1 交付检查"
run_check task1_delivery_check.sh "${delivery_args[@]}"

package_args=()
if [[ "${LIST_FILES}" == "true" ]]; then
  package_args+=(--list)
fi
if [[ "${CREATE}" == "true" ]]; then
  package_args+=(--create)
fi
if [[ "${ALLOW_WARNINGS}" == "true" ]]; then
  package_args+=(--allow-warnings)
fi

step "9/9 预览或创建最终压缩包"
run_check task1_package_preview.sh "${package_args[@]}"

echo
if [[ "${CREATE}" == "true" ]]; then
  echo "[task1-finalize] 完成：已走完最终检查和打包流程。"
else
  echo "[task1-finalize] 完成：已走完最终检查和压缩包预览；需要创建 zip 时追加 --create。"
fi
