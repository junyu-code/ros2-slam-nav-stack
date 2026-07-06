#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OVERLAY_ROOT="${WORKSPACE_DIR}/external/ros_humble_debs"
CACHE_DIR="${OVERLAY_ROOT}/cache"
OVERLAY_DIR="${OVERLAY_ROOT}/overlay"
PACKAGES=(
  ros-humble-moveit-planners-ompl
  ros-humble-moveit-simple-controller-manager
)
REQUIRED_SETUPS=(
  "${OVERLAY_DIR}/opt/ros/humble/share/moveit_planners_ompl/local_setup.bash"
  "${OVERLAY_DIR}/opt/ros/humble/share/moveit_simple_controller_manager/local_setup.bash"
)

cd "${WORKSPACE_DIR}"

mkdir -p "${CACHE_DIR}" "${OVERLAY_DIR}"

overlay_ready=true
for setup_file in "${REQUIRED_SETUPS[@]}"; do
  if [[ ! -f "${setup_file}" ]]; then
    overlay_ready=false
  fi
done

if [[ "${overlay_ready}" == "true" ]]; then
  echo "[Piper MoveIt] 已存在本地 OMPL overlay：${OVERLAY_DIR}"
  exit 0
fi

echo "[Piper MoveIt] 下载 MoveIt2 plan-only 所需插件，仅解包到 external/，不安装到系统。"
(
  cd "${CACHE_DIR}"
  for package_name in "${PACKAGES[@]}"; do
    if ! find . -maxdepth 1 -type f -name "${package_name}_*.deb" | grep -q .; then
      apt-get download "${package_name}"
    fi
  done
)

for package_name in "${PACKAGES[@]}"; do
  deb_file="$(find "${CACHE_DIR}" -maxdepth 1 -type f -name "${package_name}_*.deb" | sort | tail -n 1)"
  if [[ -z "${deb_file}" ]]; then
    echo "[Piper MoveIt] 未找到下载后的 deb：${package_name}" >&2
    exit 2
  fi

  echo "[Piper MoveIt] 解包 ${deb_file}"
  dpkg-deb -x "${deb_file}" "${OVERLAY_DIR}"
done

for setup_file in "${REQUIRED_SETUPS[@]}"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "[Piper MoveIt] 解包后仍未找到 ${setup_file}。" >&2
    exit 2
  fi
done

echo "[Piper MoveIt] 本地 MoveIt2 plan-only overlay 已准备好。"
for setup_file in "${REQUIRED_SETUPS[@]}"; do
  echo "  ${setup_file}"
done
