#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

# 只拉 AgileX open class 作为外部参考源码；external/ 已在 .gitignore 中，不进入主仓库。
OPEN_CLASS_DIR="external/agilex/agilex_open_class"
OPEN_CLASS_URL="https://github.com/agilexrobotics/agilex_open_class.git"
OPEN_CLASS_TARBALL_URL="https://codeload.github.com/agilexrobotics/agilex_open_class/tar.gz/refs/heads/master"
DOWNLOAD_TIMEOUT="${PIPER_OPEN_CLASS_DOWNLOAD_TIMEOUT:-900}"
DOWNLOAD_MODE="${PIPER_OPEN_CLASS_DOWNLOAD_MODE:-api}"
WAIT_RATE_LIMIT="${PIPER_OPEN_CLASS_WAIT_RATE_LIMIT:-0}"
CHUNK_SIZE="${PIPER_OPEN_CLASS_CHUNK_SIZE:-393216}"
USE_GIT="${PIPER_OPEN_CLASS_USE_GIT:-0}"

mkdir -p external/agilex

has_open_class_piper_packages() {
  [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/package.xml" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/urdf/piper_description.xacro" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/meshes/base_link.STL" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/meshes/link1.STL" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/meshes/link2.STL" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/meshes/link3.STL" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/meshes/link4.STL" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/meshes/link5.STL" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/meshes/link6.STL" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/meshes/link7.STL" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_description/meshes/link8.STL" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_moveit_config_v4/package.xml" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_moveit_config_v4/config/piper.srdf" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_moveit_config_v4/launch/demo.launch.py" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_moveit_config_v5/package.xml" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_moveit_config_v5/config/piper.srdf" ]] &&
    [[ -f "${OPEN_CLASS_DIR}/piper/piper_moveit_config_v5/launch/demo.launch.py" ]]
}

download_open_class_api() {
  echo "[Piper] 通过 GitHub API 只下载 Piper 官方 description / MoveIt2 配置包..."
  local downloader_args=(
    --output-dir "${OPEN_CLASS_DIR}" \
    --ref master \
    --chunk-size "${CHUNK_SIZE}"
  )
  if [[ "${WAIT_RATE_LIMIT}" == "1" || "${WAIT_RATE_LIMIT}" == "true" ]]; then
    downloader_args+=(--wait-rate-limit)
  fi
  python3 scripts/download_agilex_open_class_piper.py "${downloader_args[@]}"
}

download_open_class_tarball() {
  local tmp_tar
  tmp_tar="$(mktemp /tmp/agilex_open_class_master.XXXXXX.tar.gz)"

  echo "[Piper] 下载 AgileX open class tarball，只解压 piper 子目录..."
  echo "[Piper] 如果网络较慢，可设置 PIPER_OPEN_CLASS_DOWNLOAD_TIMEOUT=1800 后重试。"
  curl \
    --fail \
    --location \
    --retry 3 \
    --connect-timeout 20 \
    --max-time "${DOWNLOAD_TIMEOUT}" \
    --output "${tmp_tar}" \
    "${OPEN_CLASS_TARBALL_URL}"

  rm -rf "${OPEN_CLASS_DIR}"
  mkdir -p "${OPEN_CLASS_DIR}"
  tar -xzf "${tmp_tar}" \
    -C "${OPEN_CLASS_DIR}" \
    --strip-components=1 \
    --wildcards 'agilex_open_class-master/piper/*'
  rm -f "${tmp_tar}"
}

clone_open_class_sparse() {
  echo "[Piper] 稀疏克隆 AgileX open class 的 piper 子目录到 ${OPEN_CLASS_DIR}..."
  git clone \
    --depth 1 \
    --filter=blob:none \
    --sparse \
    --single-branch \
    --branch master \
    "${OPEN_CLASS_URL}" \
    "${OPEN_CLASS_DIR}"
  git -C "${OPEN_CLASS_DIR}" sparse-checkout set piper
}

if [[ -d "${OPEN_CLASS_DIR}/.git" ]]; then
  echo "[Piper] 更新 AgileX open class..."
  git -C "${OPEN_CLASS_DIR}" pull --ff-only
elif [[ "${USE_GIT}" == "1" ]]; then
  rm -rf "${OPEN_CLASS_DIR}"
  clone_open_class_sparse
elif has_open_class_piper_packages; then
  echo "[Piper] 已存在 ${OPEN_CLASS_DIR}/piper，跳过下载。"
else
  case "${DOWNLOAD_MODE}" in
    api)
      download_open_class_api
      ;;
    tarball)
      rm -rf "${OPEN_CLASS_DIR}"
      download_open_class_tarball
      ;;
    git)
      rm -rf "${OPEN_CLASS_DIR}"
      clone_open_class_sparse
      ;;
    auto)
      if ! download_open_class_api; then
        echo "[Piper] API 精准下载失败，回退到 tarball 下载。"
        rm -rf "${OPEN_CLASS_DIR}"
        download_open_class_tarball
      fi
      ;;
    *)
      echo "[Piper] 未知 PIPER_OPEN_CLASS_DOWNLOAD_MODE=${DOWNLOAD_MODE}，可选 api/tarball/git/auto。" >&2
      exit 2
      ;;
  esac
fi

if ! has_open_class_piper_packages; then
  echo "[Piper] 未找到完整 Piper 官方包，请检查 ${OPEN_CLASS_DIR}/piper。" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash

echo "[Piper] 构建官方 Piper description / MoveIt2 配置包..."
colcon build --symlink-install \
  --base-paths "${OPEN_CLASS_DIR}/piper" \
  --packages-select \
  piper_description \
  piper_moveit_config_v4 \
  piper_moveit_config_v5

echo
echo "[Piper] 官方 Piper 包构建完成。下一步请执行："
echo "  source install/setup.bash"
echo "  ros2 run slam_nav_piper_bringup piper_preflight_check.py --require-official"
echo "  ros2 launch slam_nav_piper_description piper_official_description.launch.py"
