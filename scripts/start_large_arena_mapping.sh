#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}/.."
cd "${ROOT_DIR}"
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

slam_params="${ROOT_DIR}/install/slam_nav_bringup/share/slam_nav_bringup/config/mapper_params_pcd_aligned.yaml"
map_base="${LARGE_ARENA_MAP_NAME:-large_arena_aligned}"
launch_args=()
for arg in "$@"; do
  case "${arg}" in
    map_name:=*) map_base="${arg#map_name:=}" ;;
    *) launch_args+=("${arg}") ;;
  esac
done

if [[ -z "${map_base}" || ! "${map_base}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "[large-arena-mapping] invalid map name: ${map_base}" >&2
  exit 2
fi

map_name="${map_base}"
suffix=1
while [[ -e "${ROOT_DIR}/src/slam_nav_bringup/map/${map_name}.yaml" ||
         -e "${ROOT_DIR}/src/slam_nav_bringup/map/${map_name}.pgm" ||
         -e "${ROOT_DIR}/src/slam_nav_bringup/map/${map_name}.png" ||
         -e "${ROOT_DIR}/src/FAST_LIO/PCD/${map_name}.pcd" ]]; do
  map_name="${map_base}${suffix}"
  suffix=$((suffix + 1))
done

pcd_target="${ROOT_DIR}/src/FAST_LIO/PCD/${map_name}.pcd"
mkdir -p "$(dirname "${pcd_target}")"

runtime_dir="${XDG_RUNTIME_DIR:-/tmp}"
state_file="${runtime_dir}/slam_nav_large_arena_mapping_${ROS_DOMAIN_ID:-0}.state"
mkdir -p "${runtime_dir}"
if [[ -f "${state_file}" ]]; then
  mapfile -t previous_state < "${state_file}"
  previous_pid="${previous_state[2]:-}"
  if [[ -n "${previous_pid}" ]] && kill -0 "${previous_pid}" 2>/dev/null; then
    echo "[large-arena-mapping] another mapping session is already active (pid=${previous_pid})." >&2
    exit 1
  fi
fi
printf '%s\n%s\n%s\n' "${map_name}" "${pcd_target}" "$$" > "${state_file}"

launch_pid=""
stopping=false

remove_state() {
  rm -f "${state_file}"
}

save_and_stop() {
  local signal_name="${1:-INT}"
  if [[ "${stopping}" == "true" ]]; then
    return
  fi
  stopping=true
  trap '' INT TERM HUP

  echo
  echo "[large-arena-mapping] ${signal_name} received; auto-saving 2D/PCD map pair before shutdown."
  if ! "${SCRIPT_DIR}/save_large_arena_maps.sh" --active; then
    echo "[large-arena-mapping] automatic map save failed; FAST-LIO will still try to save the PCD while exiting." >&2
  fi

  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  remove_state
  exit 0
}

trap 'save_and_stop INT' INT
trap 'save_and_stop TERM' TERM
trap 'save_and_stop HUP' HUP
trap remove_state EXIT

echo "[large-arena-mapping] map name for this run: ${map_name}"
echo "[large-arena-mapping] Ctrl+C will save both maps before stopping."

# Keep ros2 launch outside the terminal foreground process group. This lets the
# wrapper save maps while all mapping services are still alive on Ctrl+C.
setsid bash -c 'trap - INT TERM HUP; exec "$@"' bash \
  ros2 launch slam_nav_bringup mapping.launch.py \
  use_sim_time:=true \
  rviz:=true \
  slam_params:="${slam_params}" \
  pcd_map_file_path:="${pcd_target}" \
  "${launch_args[@]}" &
launch_pid=$!

if wait "${launch_pid}"; then
  launch_status=0
else
  launch_status=$?
fi
remove_state
exit "${launch_status}"
