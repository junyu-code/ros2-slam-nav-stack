#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

target_name="${1:-}"
pcd_dir="src/FAST_LIO/PCD"
default_pcd="${pcd_dir}/scan.pcd"

echo "[save_pcd_map] waiting for /map_save service..."
for _ in $(seq 1 30); do
  if ros2 service list | grep -qx "/map_save"; then
    break
  fi
  sleep 1
done

if ! ros2 service list | grep -qx "/map_save"; then
  echo "[save_pcd_map] /map_save service not found. Is FAST-LIO mapping running?" >&2
  exit 1
fi

ros2 service call /map_save std_srvs/srv/Trigger "{}"

if [[ -n "${target_name}" ]]; then
  mkdir -p "${pcd_dir}"
  if [[ ! -f "${default_pcd}" ]]; then
    echo "[save_pcd_map] expected PCD file not found: ${default_pcd}" >&2
    exit 1
  fi
  cp "${default_pcd}" "${pcd_dir}/${target_name}.pcd"
  echo "[save_pcd_map] copied ${default_pcd} -> ${pcd_dir}/${target_name}.pcd"
else
  echo "[save_pcd_map] saved to ${default_pcd}"
fi
