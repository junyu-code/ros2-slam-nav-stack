#!/usr/bin/env python3

from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).parents[1]
BRINGUP = WORKSPACE / "src" / "slam_nav_bringup"
CONFIGS = (
    BRINGUP / "config" / "nav2_params_3d.yaml",
    BRINGUP / "config" / "nav2_params_3d_rgbd.yaml",
)


class Nav2MppiConfigTest(unittest.TestCase):
    def load_params(self, path: Path) -> dict:
        with path.open(encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def test_enhanced_navigation_uses_omnidirectional_mppi(self) -> None:
        for path in CONFIGS:
            with self.subTest(config=path.name):
                params = self.load_params(path)
                controller = params["controller_server"]["ros__parameters"]["FollowPath"]

                self.assertEqual(
                    controller["plugin"],
                    "nav2_mppi_controller::MPPIController",
                )
                self.assertEqual(controller["motion_model"], "Omni")
                self.assertGreater(controller["vy_max"], 0.0)
                self.assertTrue(controller["CostCritic"]["consider_footprint"])
                self.assertIn("CostCritic", controller["critics"])
                self.assertEqual(controller["TwirlingCritic"]["cost_weight"], 8.0)

    def test_prediction_horizon_fits_inside_local_costmap(self) -> None:
        for path in CONFIGS:
            with self.subTest(config=path.name):
                params = self.load_params(path)
                controller = params["controller_server"]["ros__parameters"]["FollowPath"]
                costmap = params["local_costmap"]["local_costmap"]["ros__parameters"]

                horizon = controller["time_steps"] * controller["model_dt"]
                max_translation = max(controller["vx_max"], controller["vy_max"])
                projected_distance = horizon * max_translation
                observable_radius = min(costmap["width"], costmap["height"]) / 2.0

                self.assertLess(projected_distance, observable_radius)

    def test_bringup_declares_mppi_runtime_dependency(self) -> None:
        root = ET.parse(BRINGUP / "package.xml").getroot()
        dependencies = {node.text for node in root.findall("exec_depend")}

        self.assertIn("nav2_mppi_controller", dependencies)


if __name__ == "__main__":
    unittest.main()
