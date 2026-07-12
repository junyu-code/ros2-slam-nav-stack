#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPORT_DIR="${WORKSPACE_DIR}/tasks/task1/report_latex"
TEX_FILE="main.tex"
RUNS=2
KEEP_GOING=false

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-build-report [--once] [--keep-going]

说明：
  编译 tasks/task1/report_latex/main.tex，生成或更新 main.pdf。
  优先使用 WSL 内的 xelatex；若 WSL 没有，则自动尝试 Windows TeX Live 的 xelatex.exe。

可选参数：
  --once        只编译 1 遍；默认编译 2 遍以刷新目录和交叉引用。
  --keep-going 传给 xelatex 的 nonstopmode，遇到普通 LaTeX warning 时继续；严重错误仍会失败。

可选环境变量：
  TASK1_XELATEX_EXE=/mnt/d/texlive/2025/bin/windows/xelatex.exe
  TASK1_XELATEX_EXE='D:\texlive\2025\bin\windows\xelatex.exe'
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --once)
      RUNS=1
      ;;
    --keep-going)
      KEEP_GOING=true
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task1-report] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ ! -f "${REPORT_DIR}/${TEX_FILE}" ]]; then
  echo "[task1-report] 缺少报告源文件: ${REPORT_DIR}/${TEX_FILE}" >&2
  exit 1
fi

xelatex_args=(-interaction=nonstopmode -halt-on-error "${TEX_FILE}")
if [[ "${KEEP_GOING}" == "true" ]]; then
  xelatex_args=(-interaction=nonstopmode "${TEX_FILE}")
fi

run_linux_xelatex() {
  local xelatex_bin="$1"
  echo "[task1-report] 使用 WSL xelatex: ${xelatex_bin}"
  (
    cd "${REPORT_DIR}"
    for ((i = 1; i <= RUNS; i++)); do
      echo "[task1-report] XeLaTeX 第 ${i}/${RUNS} 遍..."
      "${xelatex_bin}" "${xelatex_args[@]}"
    done
  )
}

to_windows_path() {
  local path="$1"
  if [[ "${path}" =~ ^[A-Za-z]:\\ ]]; then
    printf '%s\n' "${path}"
  else
    wslpath -w "${path}"
  fi
}

escape_powershell_single_quote() {
  local value="$1"
  printf '%s\n' "${value//\'/\'\'}"
}

find_windows_xelatex() {
  if [[ -n "${TASK1_XELATEX_EXE:-}" ]]; then
    printf '%s\n' "${TASK1_XELATEX_EXE}"
    return 0
  fi

  local candidate
  for candidate in \
    /mnt/d/texlive/2025/bin/windows/xelatex.exe \
    /mnt/d/texlive/2024/bin/windows/xelatex.exe \
    /mnt/c/texlive/2025/bin/windows/xelatex.exe \
    /mnt/c/texlive/2024/bin/windows/xelatex.exe
  do
    if [[ -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  return 1
}

run_windows_xelatex() {
  local xelatex_exe="$1"
  local win_report_dir
  local win_xelatex
  win_report_dir="$(to_windows_path "${REPORT_DIR}")"
  win_xelatex="$(to_windows_path "${xelatex_exe}")"

  echo "[task1-report] 使用 Windows xelatex: ${win_xelatex}"
  echo "[task1-report] 报告目录: ${win_report_dir}"

  local mode="-interaction=nonstopmode -halt-on-error"
  if [[ "${KEEP_GOING}" == "true" ]]; then
    mode="-interaction=nonstopmode"
  fi

  local cmd="pushd \"${win_report_dir}\""
  local i
  for ((i = 1; i <= RUNS; i++)); do
    cmd="${cmd} && \"${win_xelatex}\" ${mode} ${TEX_FILE}"
  done
  cmd="${cmd} && popd"

  local ps_cmd
  ps_cmd="cmd.exe /d /c '$(escape_powershell_single_quote "${cmd}")'"
  powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "${ps_cmd}"
}

echo "[task1-report] 工作区: ${WORKSPACE_DIR}"
echo "[task1-report] 报告源文件: ${REPORT_DIR}/${TEX_FILE}"

if [[ -x "${SCRIPT_DIR}/task1_sync_report.sh" ]]; then
  echo "[task1-report] 同步静态避障实验表..."
  "${SCRIPT_DIR}/task1_sync_report.sh" --quiet
fi

if command -v xelatex >/dev/null 2>&1; then
  run_linux_xelatex "$(command -v xelatex)"
elif windows_xelatex="$(find_windows_xelatex)"; then
  run_windows_xelatex "${windows_xelatex}"
else
  cat >&2 <<'EOF'
[task1-report] 未找到 xelatex。
可选修复方式：
  1. 在 WSL 安装 TeX Live / xelatex；
  2. 在 Windows 安装 TeX Live，并设置 TASK1_XELATEX_EXE；
  3. 参考 tasks/task1/README.md 中的报告与检查流程。
EOF
  exit 2
fi

if [[ -f "${REPORT_DIR}/main.pdf" ]]; then
  echo "[task1-report] 已生成: ${REPORT_DIR}/main.pdf"
  ls -lh "${REPORT_DIR}/main.pdf"
else
  echo "[task1-report] 编译命令结束，但未找到 main.pdf。" >&2
  exit 1
fi
