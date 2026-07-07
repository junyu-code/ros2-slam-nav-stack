#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

STATIC_WORLD="src/slam_nav_simulation/world/nav_test_world/nav_test_world.world"
DYNAMIC_WORLD="src/slam_nav_simulation/world/nav_test_world/nav_test_world_dynamic.world"

errors=0
warnings=0

ok() {
  echo "[task1-world-check] OK: $*"
}

warn() {
  warnings=$((warnings + 1))
  echo "[task1-world-check] WARN: $*" >&2
}

fail() {
  errors=$((errors + 1))
  echo "[task1-world-check] FAIL: $*" >&2
}

check_file() {
  local path="$1"
  local desc="$2"
  if [[ -f "${path}" ]]; then
    ok "${desc}: ${path}"
  else
    fail "missing ${desc}: ${path}"
  fi
}

check_file "${STATIC_WORLD}" "static world"
check_file "${DYNAMIC_WORLD}" "dynamic world"

if command -v gz >/dev/null 2>&1; then
  for world in "${STATIC_WORLD}" "${DYNAMIC_WORLD}"; do
    if gz sdf -k "${world}" >/tmp/task1_world_check_gz.log 2>&1; then
      ok "SDF syntax check passed: ${world}"
    else
      fail "SDF syntax check failed: ${world}"
      sed 's/^/[task1-world-check]   /' /tmp/task1_world_check_gz.log >&2 || true
    fi
  done
else
  warn "gz command not found; skipped Gazebo SDF syntax check"
fi

# 这里检查任务一实际使用的旧动态场地，不再强行套用后来试做的新坡道几何。
# 目标是避免场地文件被误改后，旧地图、出生点和导航实验再次对不上。
python3 - <<'PY'
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

STATIC_WORLD = Path("src/slam_nav_simulation/world/nav_test_world/nav_test_world.world")
DYNAMIC_WORLD = Path("src/slam_nav_simulation/world/nav_test_world/nav_test_world_dynamic.world")
errors = 0
warnings = 0


def ok(message: str) -> None:
    print(f"[task1-world-check] OK: {message}")


def warn(message: str) -> None:
    global warnings
    warnings += 1
    print(f"[task1-world-check] WARN: {message}", file=sys.stderr)


def fail(message: str) -> None:
    global errors
    errors += 1
    print(f"[task1-world-check] FAIL: {message}", file=sys.stderr)


def parse_world(path: Path) -> dict[str, ET.Element]:
    try:
        root = ET.parse(path).getroot()
    except Exception as exc:  # noqa: BLE001
        fail(f"cannot parse {path}: {exc}")
        return {}
    world = root.find("world")
    if world is None:
        fail(f"{path} has no <world> element")
        return {}
    return {model.attrib.get("name", ""): model for model in world.findall("model")}


def pose_of(element: ET.Element | None) -> tuple[float, float, float, float, float, float]:
    if element is None:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    text = element.findtext("pose", default="0 0 0 0 0 0")
    values = [float(item) for item in text.split()]
    while len(values) < 6:
        values.append(0.0)
    return tuple(values[:6])


def first_link(model: ET.Element | None) -> ET.Element | None:
    if model is None:
        return None
    return model.find("link")


def geometry_size(link: ET.Element | None) -> tuple[float, float, float] | None:
    if link is None:
        return None
    size_text = link.findtext(".//collision/geometry/box/size")
    if size_text:
        values = [float(item) for item in size_text.split()]
        if len(values) == 3:
            return tuple(values)
    radius_text = link.findtext(".//collision/geometry/cylinder/radius")
    length_text = link.findtext(".//collision/geometry/cylinder/length")
    if radius_text and length_text:
        radius = float(radius_text)
        return (radius * 2.0, radius * 2.0, float(length_text))
    return None


def model_aabb(model: ET.Element | None) -> tuple[float, float, float, float] | None:
    if model is None:
        return None
    model_pose = pose_of(model)
    boxes = []
    for link in model.findall("link"):
        size = geometry_size(link)
        if size is None:
            continue
        link_pose = pose_of(link)
        cx = model_pose[0] + link_pose[0]
        cy = model_pose[1] + link_pose[1]
        yaw = model_pose[5] + link_pose[5]
        half_x = abs(math.cos(yaw)) * size[0] / 2.0 + abs(math.sin(yaw)) * size[1] / 2.0
        half_y = abs(math.sin(yaw)) * size[0] / 2.0 + abs(math.cos(yaw)) * size[1] / 2.0
        boxes.append((cx - half_x, cx + half_x, cy - half_y, cy + half_y))
    if not boxes:
        return None
    return (
        min(item[0] for item in boxes),
        max(item[1] for item in boxes),
        min(item[2] for item in boxes),
        max(item[3] for item in boxes),
    )


def close_tuple(a: tuple[float, ...] | None, b: tuple[float, ...] | None, eps: float = 1e-6) -> bool:
    if a is None or b is None or len(a) != len(b):
        return False
    return all(abs(x - y) <= eps for x, y in zip(a, b))


def check_required(models: dict[str, ET.Element], label: str) -> None:
    required = [
        "nav_test_floor",
        "wall_north",
        "wall_south",
        "wall_west",
        "wall_east",
        "corridor_wall_left",
        "corridor_wall_right",
        "narrow_gate_top",
        "narrow_gate_bottom",
        "low_obstacle_1",
        "low_obstacle_2",
        "column_1",
        "column_2",
        "ramp",
    ]
    missing = [name for name in required if name not in models]
    if missing:
        fail(f"{label} missing models: {', '.join(missing)}")
    else:
        ok(f"{label} required static models present: {len(required)}")

    for name in required:
        model = models.get(name)
        if model is None:
            continue
        if model.findtext("static", default="false").strip().lower() != "true":
            fail(f"{label} model should be static=true: {name}")


def check_floor_bounds(models: dict[str, ET.Element], label: str) -> None:
    floor = models.get("nav_test_floor")
    floor_size = geometry_size(first_link(floor))
    floor_pose = pose_of(floor)
    if floor_size is None:
        fail(f"{label} floor has no box collision size")
        return
    xmin = floor_pose[0] - floor_size[0] / 2.0
    xmax = floor_pose[0] + floor_size[0] / 2.0
    ymin = floor_pose[1] - floor_size[1] / 2.0
    ymax = floor_pose[1] + floor_size[1] / 2.0
    for name in ("ramp", "low_obstacle_1", "low_obstacle_2", "column_1", "column_2"):
        bounds = model_aabb(models.get(name))
        if bounds is None:
            continue
        if bounds[0] < xmin - 1e-6 or bounds[1] > xmax + 1e-6 or bounds[2] < ymin - 1e-6 or bounds[3] > ymax + 1e-6:
            fail(f"{label} {name} outside floor bounds: {bounds}")
        else:
            ok(f"{label} {name} inside floor bounds: {tuple(round(v, 3) for v in bounds)}")


def check_static_consistency(static_models: dict[str, ET.Element], dynamic_models: dict[str, ET.Element]) -> None:
    # ramp 在旧静态/动态场地中使用了不同表达形式；这里保留历史兼容性，不把它当错误。
    comparable = [
        "nav_test_floor",
        "wall_north",
        "wall_south",
        "wall_west",
        "wall_east",
        "corridor_wall_left",
        "corridor_wall_right",
        "narrow_gate_top",
        "narrow_gate_bottom",
        "low_obstacle_1",
        "low_obstacle_2",
        "column_1",
        "column_2",
    ]
    for name in comparable:
        s_model = static_models.get(name)
        d_model = dynamic_models.get(name)
        if s_model is None or d_model is None:
            continue
        if not close_tuple(pose_of(s_model), pose_of(d_model)):
            fail(f"static/dynamic pose mismatch: {name}")
        if not close_tuple(geometry_size(first_link(s_model)), geometry_size(first_link(d_model))):
            fail(f"static/dynamic geometry mismatch: {name}")
    ok("static and dynamic worlds share the same fixed obstacle layout")


def check_dynamic_obstacles(models: dict[str, ET.Element]) -> None:
    for name in ("moving_obstacle", "fast_moving_obstacle"):
        model = models.get(name)
        if model is None:
            fail(f"dynamic world missing obstacle: {name}")
            continue
        if model.findtext("static", default="true").strip().lower() != "false":
            fail(f"dynamic obstacle should be static=false: {name}")
        plugin = next(
            (
                item
                for item in model.findall("plugin")
                if item.attrib.get("filename") == "libdynamic_obstacle_plugin.so"
            ),
            None,
        )
        if plugin is None:
            fail(f"dynamic obstacle missing libdynamic_obstacle_plugin.so: {name}")
            continue
        values = {}
        for key in ("start_x", "start_y", "end_x", "end_y", "speed", "yield_radius", "yield_resume_radius"):
            text = plugin.findtext(key)
            if text is None:
                fail(f"dynamic obstacle {name} missing plugin field: {key}")
                continue
            values[key] = float(text)
        robot_model = plugin.findtext("robot_model", default="").strip()
        if robot_model != "mobile_robot":
            fail(f"dynamic obstacle {name} robot_model should be mobile_robot")
        if len(values) == 7:
            distance = math.hypot(values["end_x"] - values["start_x"], values["end_y"] - values["start_y"])
            if distance < 0.5:
                fail(f"dynamic obstacle path too short: {name}")
            elif values["speed"] <= 0.0:
                fail(f"dynamic obstacle speed must be positive: {name}")
            elif values["yield_radius"] <= 0.0:
                fail(f"dynamic obstacle yield_radius must be positive: {name}")
            elif values["yield_resume_radius"] < values["yield_radius"]:
                fail(f"dynamic obstacle yield_resume_radius must be >= yield_radius: {name}")
            else:
                ok(
                    f"dynamic obstacle {name} path valid, "
                    f"speed={values['speed']:.2f} m/s, yield={values['yield_radius']:.2f} m"
                )


static_models = parse_world(STATIC_WORLD)
dynamic_models = parse_world(DYNAMIC_WORLD)
for label, models in (("static world", static_models), ("dynamic world", dynamic_models)):
    check_required(models, label)
    check_floor_bounds(models, label)
check_static_consistency(static_models, dynamic_models)
check_dynamic_obstacles(dynamic_models)

print(f"[task1-world-check] geometry summary: errors={errors}, warnings={warnings}")
if errors:
    sys.exit(1)
PY

python_status=$?
if [[ ${python_status} -ne 0 ]]; then
  errors=$((errors + 1))
fi

echo "[task1-world-check] summary: errors=${errors}, warnings=${warnings}"
if [[ ${errors} -gt 0 ]]; then
  exit 1
fi
