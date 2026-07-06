#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

CREATE=false
ALLOW_WARNINGS=false
LIST_FILES=false
OUTPUT_DIR="dist"
PACKAGE_NAME="3232072072234+佘俊谕.zip"

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-package-preview [--list]
  ./run.sh task1-package-preview --create [--allow-warnings]

说明：
  默认只预览 task1 最终压缩包会包含的文件数量和估算体积，不创建文件。
  --list            列出将被打包的文件路径。
  --create          创建 zip 到 dist/ 目录。
  --allow-warnings  允许在 task1-delivery-check --strict 未通过时创建草稿包。
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
    --list)
      LIST_FILES=true
      ;;
    --output-dir)
      shift
      OUTPUT_DIR="${1:?--output-dir 需要路径}"
      ;;
    --name)
      shift
      PACKAGE_NAME="${1:?--name 需要文件名}"
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task1-package] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

INCLUDE_PATHS=(
  run.sh
  scripts
  src
  README.md
  PROJECT_PROCESS.md
  tasks/task1
)

should_skip() {
  local path="$1"
  case "${path}" in
    build/*|install/*|log/*|.git/*|.vscode/*|dist/*)
      return 0
      ;;
    datasets/*|models/*|checkpoints/*|runs/*|wandb/*)
      return 0
      ;;
    external/*|third_party/*|vendor/*)
      return 0
      ;;
    */__pycache__/*|*.pyc)
      return 0
      ;;
    *.bag|*.db3|*.mcap)
      return 0
      ;;
    *.pcd|*.ply|*.las|*.laz)
      return 0
      ;;
    *.onnx|*.pt|*.pth|*.ckpt|*.engine|*.safetensors)
      return 0
      ;;
    *.aux|*.log|*.out|*.toc|*.fls|*.fdb_latexmk|*.synctex.gz)
      return 0
      ;;
    src/FAST_LIO/PCD/*|src/FAST_LIO/Log/*.txt)
      return 0
      ;;
  esac
  return 1
}

tmp_list="$(mktemp)"
cleanup() {
  rm -f "${tmp_list}"
}
trap cleanup EXIT

missing=0
for path in "${INCLUDE_PATHS[@]}"; do
  if [[ ! -e "${path}" ]]; then
    echo "[task1-package] 缺少建议打包路径: ${path}" >&2
    missing=$((missing + 1))
  fi
done
if [[ "${missing}" -ne 0 ]]; then
  exit 1
fi

while IFS= read -r file; do
  if should_skip "${file}"; then
    continue
  fi
  printf '%s\n' "${file}" >> "${tmp_list}"
done < <(find "${INCLUDE_PATHS[@]}" -type f | sort)

file_count="$(wc -l < "${tmp_list}" | tr -d ' ')"
total_bytes=0
while IFS= read -r file; do
  size="$(stat -c '%s' "${file}" 2>/dev/null || echo 0)"
  total_bytes=$((total_bytes + size))
done < "${tmp_list}"
if command -v numfmt >/dev/null 2>&1; then
  human_size="$(numfmt --to=iec --suffix=B "${total_bytes}")"
else
  human_size="${total_bytes}B"
fi

echo "[task1-package] 工作区: ${WORKSPACE_DIR}"
echo "[task1-package] 建议压缩包名: ${PACKAGE_NAME}"
echo "[task1-package] 将包含文件数: ${file_count}"
echo "[task1-package] 估算体积: ${human_size}"
echo "[task1-package] 包含根路径:"
printf '  %s\n' "${INCLUDE_PATHS[@]}"
echo "[task1-package] 已排除: build/ install/ log/ .git/ .vscode/ dist/ datasets/ external/ third_party/ vendor/ rosbag/点云/模型权重/LaTeX 辅助文件"

if [[ "${LIST_FILES}" == "true" ]]; then
  echo "[task1-package] 文件列表:"
  sed 's/^/  /' "${tmp_list}"
fi

if [[ "${CREATE}" != "true" ]]; then
  echo "[task1-package] 预览完成；未创建 zip。需要创建时运行：./run.sh task1-package-preview --create"
  exit 0
fi

if [[ "${ALLOW_WARNINGS}" != "true" ]]; then
  echo "[task1-package] --create 前先执行严格交付检查。"
  if ! "${SCRIPT_DIR}/task1_delivery_check.sh" --strict; then
    echo "[task1-package] 严格交付检查未通过，已停止创建 zip。补齐材料后重试，或用 --allow-warnings 创建草稿包。" >&2
    exit 2
  fi
fi

if ! command -v zip >/dev/null 2>&1; then
  echo "[task1-package] 系统缺少 zip 命令，无法创建压缩包；仍可按上方文件列表手动打包。" >&2
  exit 2
fi

mkdir -p "${OUTPUT_DIR}"
output_path="${OUTPUT_DIR}/${PACKAGE_NAME}"
rm -f "${output_path}"
zip -q -@ "${output_path}" < "${tmp_list}"
echo "[task1-package] 已创建: ${output_path}"
