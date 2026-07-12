from __future__ import annotations

import copy
import tempfile
from pathlib import Path
from typing import Any

import yaml


BUILTIN_PROFILES = ("omni", "diff_drive", "go2")


class BaseProfileError(ValueError):
    """Raised when a chassis profile cannot safely configure Nav2."""


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise BaseProfileError(f"failed to load YAML file {path}: {exc}") from exc

    if not isinstance(data, dict):
        raise BaseProfileError(f"YAML file must contain a mapping: {path}")
    return data


def deep_merge(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    """Return a recursive mapping merge without mutating either input."""
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def resolve_base_profile_file(
    bringup_share: str | Path,
    profile_name: str,
    custom_profile_file: str | Path | None = None,
) -> Path:
    custom = str(custom_profile_file or "").strip()
    if custom:
        path = Path(custom).expanduser().resolve()
    else:
        if profile_name not in BUILTIN_PROFILES:
            choices = ", ".join(BUILTIN_PROFILES)
            raise BaseProfileError(
                f"unknown base profile '{profile_name}', expected one of: {choices}"
            )
        path = Path(bringup_share) / "config" / "base_profiles" / f"{profile_name}.yaml"

    if not path.is_file():
        raise BaseProfileError(f"base profile does not exist: {path}")
    return path


def load_base_profile(path: str | Path) -> dict[str, Any]:
    profile = _load_yaml(Path(path))
    validate_base_profile(profile, Path(path))
    return profile


def validate_base_profile(profile: dict[str, Any], path: Path | None = None) -> None:
    label = str(path) if path else "base profile"
    try:
        controller = profile["controller_server"]["ros__parameters"]["FollowPath"]
        smoother = profile["velocity_smoother"]["ros__parameters"]
        local_costmap = profile["local_costmap"]["local_costmap"]["ros__parameters"]
        global_costmap = profile["global_costmap"]["global_costmap"]["ros__parameters"]
        safe_bridge = profile["safe_cmd_bridge_node"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise BaseProfileError(f"{label} is missing required section: {exc}") from exc

    motion_model = controller.get("motion_model")
    if motion_model not in ("Omni", "DiffDrive"):
        raise BaseProfileError(f"{label} has unsupported motion_model: {motion_model}")

    for key in ("max_velocity", "min_velocity", "max_accel", "max_decel"):
        value = smoother.get(key)
        if not isinstance(value, list) or len(value) != 3:
            raise BaseProfileError(f"{label} velocity_smoother.{key} must have 3 values")

    if local_costmap.get("footprint") != global_costmap.get("footprint"):
        raise BaseProfileError(f"{label} local and global footprints must match")

    vy_max = float(controller.get("vy_max", 0.0))
    max_vy = float(smoother["max_velocity"][1])
    min_vy = float(smoother["min_velocity"][1])
    max_ay = float(smoother["max_accel"][1])
    min_ay = float(smoother["max_decel"][1])
    safe_max_vy = float(safe_bridge.get("max_vy", 0.0))
    safe_min_vy = float(safe_bridge.get("min_vy", 0.0))

    if motion_model == "DiffDrive" and any(
        abs(value) > 1e-9
        for value in (vy_max, max_vy, min_vy, max_ay, min_ay, safe_max_vy, safe_min_vy)
    ):
        raise BaseProfileError(f"{label} DiffDrive profile must set every y-axis limit to 0")
    if motion_model == "Omni" and min(vy_max, max_vy, safe_max_vy) <= 0.0:
        raise BaseProfileError(f"{label} Omni profile must allow positive y velocity")


def build_profiled_nav2_params(
    nav2_params_file: str | Path,
    base_profile_file: str | Path,
) -> Path:
    base = _load_yaml(Path(nav2_params_file))
    profile = load_base_profile(base_profile_file)
    merged = deep_merge(base, profile)

    output = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="slam_nav_profiled_",
        suffix=".yaml",
        delete=False,
    )
    with output:
        yaml.safe_dump(merged, output, sort_keys=False)
    return Path(output.name)
