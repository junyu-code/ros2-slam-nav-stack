#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

# Piper 后续会产生厂家源码、训练集、模型权重、rosbag 和点云。
# 这个检查只守住未来新增边界；仓库里已有的非 Piper 大文件先只提示 warning。
MAX_TRACKED_BYTES="${PIPER_REPO_MAX_TRACKED_BYTES:-52428800}"   # 50 MiB
MAX_PIPER_BYTES="${PIPER_REPO_MAX_PIPER_BYTES:-10485760}"       # 10 MiB
WARN_TRACKED_BYTES="${PIPER_REPO_WARN_TRACKED_BYTES:-26214400}" # 25 MiB

required_gitignore_patterns=(
  'external/'
  'third_party/'
  'vendor/'
  'datasets/'
  'models/'
  'checkpoints/'
  'runs/'
  'wandb/'
  'bags/'
  'rosbags/'
  'weights/'
  'artifacts/'
  'outputs/'
  '*.bag'
  '*.db3'
  '*.mcap'
  '*.onnx'
  '*.pt'
  '*.pth'
  '*.ckpt'
  '*.engine'
  '*.safetensors'
  '*.npy'
  '*.npz'
  '*.tfrecord'
  '*.pcd'
  '*.ply'
  '*.las'
  '*.laz'
  '*.mp4'
  '*.avi'
  '*.mov'
  '*.mkv'
)

forbidden_path_regex='^(external|third_party|vendor|datasets|models|checkpoints|runs|wandb|bags|rosbags|weights|artifacts|outputs)/'
forbidden_ext_regex='\.(bag|db3|mcap|onnx|pt|pth|ckpt|engine|safetensors|npy|npz|tfrecord|pcd|ply|las|laz|mp4|avi|mov|mkv|7z|tar|tar\.gz)$'

failures=()
warnings=()

for pattern in "${required_gitignore_patterns[@]}"; do
  if ! grep -qxF "${pattern}" .gitignore; then
    failures+=(".gitignore 缺少规则：${pattern}")
  fi
done

while IFS= read -r tracked_path; do
  [[ -z "${tracked_path}" ]] && continue
  if [[ "${tracked_path}" =~ ${forbidden_path_regex} ]]; then
    failures+=("禁止跟踪的大目录文件：${tracked_path}")
  fi
  if [[ "${tracked_path}" =~ ${forbidden_ext_regex} ]]; then
    failures+=("禁止跟踪的数据/模型产物：${tracked_path}")
  fi
done < <(git ls-files)

while IFS= read -r -d '' tracked_path; do
  [[ -f "${tracked_path}" ]] || continue
  size_bytes="$(stat -c '%s' "${tracked_path}")"
  if (( size_bytes > MAX_TRACKED_BYTES )); then
    failures+=("单个 Git 跟踪文件超过 $((MAX_TRACKED_BYTES / 1024 / 1024)) MiB：${tracked_path} (${size_bytes} bytes)")
  elif (( size_bytes > WARN_TRACKED_BYTES )); then
    warnings+=("现有大文件 warning：${tracked_path} (${size_bytes} bytes)")
  fi

  if [[ "${tracked_path}" == src/slam_nav_piper_* ]] && (( size_bytes > MAX_PIPER_BYTES )); then
    failures+=("Piper 包内文件超过 $((MAX_PIPER_BYTES / 1024 / 1024)) MiB：${tracked_path} (${size_bytes} bytes)")
  fi
done < <(git ls-files -z)

echo "[Piper Size] GitHub 体积边界检查"
echo "[Piper Size] 阈值：tracked>$((MAX_TRACKED_BYTES / 1024 / 1024)) MiB 失败，Piper 包内>$((MAX_PIPER_BYTES / 1024 / 1024)) MiB 失败。"

if [[ -d external ]]; then
  external_size="$(du -sh external 2>/dev/null | awk '{print $1}')"
  echo "[Piper Size] external/ 当前大小：${external_size:-unknown}（已由 .gitignore 排除，不应进入 Git）"
fi

if (( ${#warnings[@]} > 0 )); then
  echo
  echo "[Piper Size] Warning：以下是当前仓库已有的大文件，Piper 扩展不应再新增类似产物："
  for item in "${warnings[@]}"; do
    echo "  - ${item}"
  done
fi

if (( ${#failures[@]} > 0 )); then
  echo
  echo "[Piper Size] FAIL：发现会撑大 GitHub 仓库或破坏 Piper 外部依赖边界的文件："
  for item in "${failures[@]}"; do
    echo "  - ${item}"
  done
  exit 2
fi

echo
echo "[Piper Size] OK：Piper 外部依赖、学习数据、模型权重、rosbag 和点云产物没有进入 Git 跟踪。"
