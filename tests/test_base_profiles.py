#!/usr/bin/env python3

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).parents[1]
BRINGUP = WORKSPACE / "src" / "slam_nav_bringup"
sys.path.insert(0, str(BRINGUP))

from slam_nav_bringup.base_profiles import (  # noqa: E402
    BUILTIN_PROFILES,
    build_profiled_nav2_params,
    load_base_profile,
    resolve_base_profile_file,
)


class BaseProfileTest(unittest.TestCase):
    def test_builtin_profiles_are_valid_and_preserve_mppi(self) -> None:
        nav2_configs = (
            BRINGUP / "config" / "nav2_params_3d.yaml",
            BRINGUP / "config" / "nav2_params_3d_rgbd.yaml",
        )

        for profile_name in BUILTIN_PROFILES:
            profile_path = resolve_base_profile_file(BRINGUP, profile_name)
            load_base_profile(profile_path)
            for nav2_config in nav2_configs:
                with self.subTest(profile=profile_name, config=nav2_config.name):
                    merged_path = build_profiled_nav2_params(nav2_config, profile_path)
                    try:
                        merged = yaml.safe_load(merged_path.read_text(encoding="utf-8"))
                    finally:
                        merged_path.unlink(missing_ok=True)

                    controller = merged["controller_server"]["ros__parameters"]["FollowPath"]
                    self.assertEqual(
                        controller["plugin"],
                        "nav2_mppi_controller::MPPIController",
                    )
                    self.assertTrue(controller["CostCritic"]["consider_footprint"])

    def test_profile_limits_are_consistent_across_output_layers(self) -> None:
        for profile_name in BUILTIN_PROFILES:
            with self.subTest(profile=profile_name):
                profile = load_base_profile(
                    resolve_base_profile_file(BRINGUP, profile_name)
                )
                controller = profile["controller_server"]["ros__parameters"]["FollowPath"]
                smoother = profile["velocity_smoother"]["ros__parameters"]
                safe = profile["safe_cmd_bridge_node"]["ros__parameters"]

                self.assertEqual(controller["vx_max"], smoother["max_velocity"][0])
                self.assertEqual(controller["vx_min"], smoother["min_velocity"][0])
                self.assertEqual(controller["vy_max"], smoother["max_velocity"][1])
                self.assertEqual(controller["wz_max"], smoother["max_velocity"][2])
                self.assertEqual(safe["max_vx"], smoother["max_velocity"][0])
                self.assertEqual(safe["min_vx"], smoother["min_velocity"][0])
                self.assertEqual(safe["max_vy"], smoother["max_velocity"][1])
                self.assertEqual(safe["min_vy"], smoother["min_velocity"][1])
                self.assertEqual(safe["max_wz"], smoother["max_velocity"][2])
                self.assertEqual(safe["min_wz"], smoother["min_velocity"][2])

    def test_diff_drive_profiles_disable_lateral_motion(self) -> None:
        for profile_name in ("diff_drive", "go2"):
            with self.subTest(profile=profile_name):
                profile = load_base_profile(
                    resolve_base_profile_file(BRINGUP, profile_name)
                )
                controller = profile["controller_server"]["ros__parameters"]["FollowPath"]
                smoother = profile["velocity_smoother"]["ros__parameters"]

                self.assertEqual(controller["motion_model"], "DiffDrive")
                self.assertEqual(controller["vy_max"], 0.0)
                self.assertEqual(smoother["max_velocity"][1], 0.0)
                self.assertEqual(smoother["min_velocity"][1], 0.0)
                self.assertEqual(smoother["max_accel"][1], 0.0)
                self.assertEqual(smoother["max_decel"][1], 0.0)

    def test_custom_profile_file_overrides_named_profile(self) -> None:
        custom = BRINGUP / "config" / "base_profiles" / "diff_drive.yaml"

        resolved = resolve_base_profile_file(BRINGUP, "omni", custom)

        self.assertEqual(resolved, custom.resolve())


if __name__ == "__main__":
    unittest.main()
