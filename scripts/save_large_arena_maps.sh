#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${SCRIPT_DIR}/.."
cd "${ROOT_DIR}"
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

runtime_dir="${XDG_RUNTIME_DIR:-/tmp}"
state_file="${runtime_dir}/slam_nav_large_arena_mapping_${ROS_DOMAIN_ID:-0}.state"
active_only=false
if [[ "${1:-}" == "--active" ]]; then
  active_only=true
  shift
fi

active_name=""
active_pcd=""
active_pid=""
if [[ -f "${state_file}" ]]; then
  mapfile -t active_state < "${state_file}"
  active_name="${active_state[0]:-}"
  active_pcd="${active_state[1]:-}"
  active_pid="${active_state[2]:-}"
  if [[ -z "${active_pid}" ]] || ! kill -0 "${active_pid}" 2>/dev/null; then
    active_name=""
    active_pcd=""
    active_pid=""
    rm -f "${state_file}"
  fi
fi

if [[ "${active_only}" == "true" ]]; then
  if [[ -z "${active_name}" || -z "${active_pcd}" ]]; then
    echo "[save-large-arena-maps] active mapping state is unavailable." >&2
    exit 1
  fi
  name="${active_name}"
elif [[ -n "${1:-}" ]]; then
  name="$1"
elif [[ -n "${active_name}" ]]; then
  name="${active_name}"
else
  name="large_arena_aligned"
fi

if [[ -z "${name}" || ! "${name}" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  echo "[save-large-arena-maps] invalid map name: ${name}" >&2
  exit 2
fi

# The active run already selected a collision-free name. For standalone or
# custom saves, find the first suffix that does not overwrite an existing pair.
if [[ "${name}" != "${active_name}" ]]; then
  name_base="${name}"
  suffix=1
  while [[ -e "src/slam_nav_bringup/map/${name}.yaml" ||
           -e "src/slam_nav_bringup/map/${name}.pgm" ||
           -e "src/slam_nav_bringup/map/${name}.png" ||
           -e "src/FAST_LIO/PCD/${name}.pcd" ]]; do
    name="${name_base}${suffix}"
    suffix=$((suffix + 1))
  done
fi

map_prefix="src/slam_nav_bringup/map/${name}"
pcd_target="src/FAST_LIO/PCD/${name}.pcd"
mapping_pcd="${active_pcd:-src/FAST_LIO/PCD/large_arena_aligned.pcd}"

mkdir -p src/slam_nav_bringup/map src/FAST_LIO/PCD
echo "[save-large-arena-maps] saving FAST-LIO PCD"
pcd_saved=false
pcd_save_output=""
if pcd_save_output="$(
  timeout --foreground 300 ros2 service call /map_save std_srvs/srv/Trigger "{}"
)"; then
  printf '%s\n' "${pcd_save_output}"
fi
if [[ "${pcd_save_output}" == *"success=True"* && -s "${mapping_pcd}" ]]; then
  if [[ ! "${mapping_pcd}" -ef "${pcd_target}" ]]; then
    cp "${mapping_pcd}" "${pcd_target}"
  fi
  pcd_saved=true
else
  echo "[save-large-arena-maps] FAST-LIO service did not save ${mapping_pcd}; shutdown fallback will still be attempted." >&2
fi

echo "[save-large-arena-maps] saving 2D map: ${map_prefix}.{yaml,pgm}"
map_prefix_abs="${ROOT_DIR}/${map_prefix}"
slam_save_output=""
if slam_save_output="$(
  timeout --foreground 30 ros2 service call \
    /slam_toolbox/save_map \
    slam_toolbox/srv/SaveMap \
    "{name: {data: '${map_prefix_abs}'}}"
)"; then
  printf '%s\n' "${slam_save_output}"
fi

map_saved=false
if [[ "${slam_save_output}" == *"result=0"* &&
      -s "${map_prefix}.yaml" &&
      -s "${map_prefix}.pgm" ]]; then
  map_saved=true
else
  echo "[save-large-arena-maps] slam_toolbox save service did not produce a map; trying /map subscription fallback."
  if timeout --foreground 30 ros2 run nav2_map_server map_saver_cli \
    -f "${map_prefix}" \
    --ros-args -p save_map_timeout:=10.0; then
    if [[ -s "${map_prefix}.yaml" && -s "${map_prefix}.pgm" ]]; then
      map_saved=true
    fi
  fi
fi

if [[ "${map_saved}" != "true" ]]; then
  echo "[save-large-arena-maps] failed to save the 2D map; check /scan, TF, and slam_toolbox." >&2
fi
if [[ "${pcd_saved}" != "true" || "${map_saved}" != "true" ]]; then
  exit 1
fi

echo "[save-large-arena-maps] saved aligned map pair:"
echo "  2D: ${map_prefix}.yaml"
echo "  3D: ${pcd_target}"
echo "[save-large-arena-maps] test with:"
echo "  ./run.sh large-arena-nav map:=${ROOT_DIR}/${map_prefix}.yaml map_pcd_path:=${ROOT_DIR}/${pcd_target}"
