#!/usr/bin/env python3

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


WORKSPACE = Path(__file__).parents[1]
BRINGUP = WORKSPACE / "src" / "slam_nav_bringup"
sys.path.insert(0, str(BRINGUP))

from slam_nav_bringup.localization_consensus import (  # noqa: E402
    AMCL_RESET_RECOMMENDED,
    DISAGREEMENT_OBSERVED,
    DISAGREEMENT_UNRESOLVED,
    FAST_LIO_GLOBAL_CORRECTION_RECOMMENDED,
    GICP_REJECTED,
    INSUFFICIENT_DATA,
    MANUAL_RELOCALIZATION_REQUIRED,
    NORMAL,
    Pose2D,
    compose_pose,
    evaluate_consensus,
    pose_delta,
)


class LocalizationConsensusTest(unittest.TestCase):
    def evaluate(self, fast_lio, amcl, gicp, **health):
        return evaluate_consensus(
            fast_lio,
            amcl,
            gicp,
            fast_lio_healthy=health.get("fast_lio_healthy", True),
            amcl_healthy=health.get("amcl_healthy", True),
            gicp_healthy=health.get("gicp_healthy", True),
        )

    def test_pose_composition_uses_parent_rotation(self):
        result = compose_pose(Pose2D(1.0, 2.0, math.pi / 2.0), Pose2D(2.0, 0.0, 0.1))

        self.assertAlmostEqual(result.x, 1.0)
        self.assertAlmostEqual(result.y, 4.0)
        self.assertAlmostEqual(result.yaw, math.pi / 2.0 + 0.1)

    def test_yaw_difference_wraps_at_pi(self):
        delta = pose_delta(Pose2D(0.0, 0.0, math.pi - 0.05), Pose2D(0.0, 0.0, -math.pi + 0.05))

        self.assertAlmostEqual(delta.yaw, 0.1)

    def test_all_sources_agree(self):
        decision = self.evaluate(
            Pose2D(1.0, 2.0, 0.1),
            Pose2D(1.1, 2.0, 0.12),
            Pose2D(1.05, 1.95, 0.08),
        )

        self.assertEqual(decision.decision, NORMAL)

    def test_fast_lio_and_gicp_can_recommend_amcl_reset(self):
        decision = self.evaluate(
            Pose2D(1.0, 2.0, 0.1),
            Pose2D(1.8, 2.0, 0.5),
            Pose2D(1.05, 2.0, 0.12),
            amcl_healthy=False,
        )

        self.assertEqual(decision.decision, AMCL_RESET_RECOMMENDED)
        self.assertEqual(decision.reference, "gicp")

    def test_amcl_and_gicp_can_recommend_global_correction(self):
        decision = self.evaluate(
            Pose2D(1.0, 2.0, 0.1),
            Pose2D(1.8, 2.0, 0.5),
            Pose2D(1.75, 2.0, 0.48),
            fast_lio_healthy=False,
        )

        self.assertEqual(decision.decision, FAST_LIO_GLOBAL_CORRECTION_RECOMMENDED)

    def test_fast_lio_and_amcl_reject_gicp_outlier(self):
        decision = self.evaluate(
            Pose2D(1.0, 2.0, 0.1),
            Pose2D(1.05, 2.0, 0.12),
            Pose2D(1.8, 2.0, 0.5),
            gicp_healthy=False,
        )

        self.assertEqual(decision.decision, GICP_REJECTED)

    def test_small_disagreement_is_observation_only(self):
        decision = self.evaluate(
            Pose2D(1.0, 2.0, 0.1),
            Pose2D(1.35, 2.0, 0.1),
            Pose2D(1.05, 2.0, 0.1),
        )

        self.assertEqual(decision.decision, DISAGREEMENT_OBSERVED)

    def test_unhealthy_agreeing_references_do_not_recommend_action(self):
        decision = self.evaluate(
            Pose2D(1.0, 2.0, 0.1),
            Pose2D(1.8, 2.0, 0.5),
            Pose2D(1.05, 2.0, 0.12),
            fast_lio_healthy=False,
        )

        self.assertEqual(decision.decision, DISAGREEMENT_UNRESOLVED)

    def test_large_disagreement_requires_manual_relocalization(self):
        decision = self.evaluate(
            Pose2D(1.0, 2.0, 0.1),
            Pose2D(4.0, 2.0, 0.5),
            Pose2D(1.05, 2.0, 0.12),
        )

        self.assertEqual(decision.decision, MANUAL_RELOCALIZATION_REQUIRED)

    def test_missing_candidate_is_insufficient(self):
        decision = self.evaluate(Pose2D(1.0, 2.0, 0.1), None, Pose2D(1.0, 2.0, 0.1))

        self.assertEqual(decision.decision, INSUFFICIENT_DATA)


if __name__ == "__main__":
    unittest.main()
