#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "preprocess_occupancy_map.py"
SPEC = importlib.util.spec_from_file_location("preprocess_occupancy_map", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class OccupancyMapFilterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.thresholds = MODULE.MapThresholds(occupied=0.65, free=0.25, negate=False)

    def test_unknown_encoding_round_trips_through_thresholds(self) -> None:
        occupied = np.array([[True, False, False]])
        free = np.array([[False, True, False]])
        unknown = np.array([[False, False, True]])

        encoded = MODULE.encode_map(occupied, free, unknown, self.thresholds)
        decoded_occupied, decoded_free, decoded_unknown = MODULE.classify_map(
            encoded,
            self.thresholds,
        )

        np.testing.assert_array_equal(decoded_occupied, occupied)
        np.testing.assert_array_equal(decoded_free, free)
        np.testing.assert_array_equal(decoded_unknown, unknown)
        self.assertNotEqual(int(encoded[0, 2]), 205)

    def test_standard_unknown_gray_is_unknown_with_ros_threshold(self) -> None:
        thresholds = MODULE.MapThresholds(occupied=0.65, free=0.196, negate=False)

        occupied, free, unknown = MODULE.classify_map(
            np.array([[205]], dtype=np.uint8),
            thresholds,
        )

        self.assertFalse(occupied[0, 0])
        self.assertFalse(free[0, 0])
        self.assertTrue(unknown[0, 0])

    def test_small_enclosed_free_hole_is_filled(self) -> None:
        occupied = np.ones((7, 7), dtype=bool)
        occupied[3, 3] = False
        unknown = np.zeros_like(occupied)

        filtered, filled = MODULE.fill_small_free_holes(occupied, unknown, 1)

        self.assertTrue(filtered[3, 3])
        self.assertTrue(filled[3, 3])

    def test_unknown_hole_is_preserved(self) -> None:
        occupied = np.ones((7, 7), dtype=bool)
        occupied[3, 3] = False
        unknown = np.zeros_like(occupied)
        unknown[3, 3] = True

        filtered, filled = MODULE.fill_small_free_holes(occupied, unknown, 4)

        self.assertFalse(filtered[3, 3])
        self.assertFalse(filled[3, 3])

    def test_single_obstacle_cell_is_removed(self) -> None:
        occupied = np.zeros((7, 7), dtype=bool)
        occupied[3, 3] = True

        filtered, removed = MODULE.remove_small_obstacle_groups(occupied, 2)

        self.assertFalse(filtered[3, 3])
        self.assertTrue(removed[3, 3])

    def test_closing_does_not_overwrite_unknown(self) -> None:
        occupied = np.zeros((7, 7), dtype=bool)
        occupied[3, 2] = True
        occupied[3, 4] = True
        unknown = np.zeros_like(occupied)
        unknown[3, 3] = True

        filtered, added = MODULE.close_obstacle_mask(occupied, unknown, 1)

        self.assertFalse(filtered[3, 3])
        self.assertFalse(added[3, 3])

    def test_closing_never_removes_existing_obstacles(self) -> None:
        occupied = np.zeros((7, 7), dtype=bool)
        occupied[0, 0] = True
        occupied[3, 2:5] = True
        unknown = np.zeros_like(occupied)

        filtered, _ = MODULE.close_obstacle_mask(occupied, unknown, 1)

        self.assertTrue(np.all(filtered[occupied]))

    def test_free_space_opening_removes_thin_rays(self) -> None:
        free = np.zeros((9, 9), dtype=bool)
        free[2:7, 2:7] = True
        free[4, 7:9] = True

        filtered, removed = MODULE.open_free_space_mask(free, 1)

        self.assertTrue(filtered[4, 4])
        self.assertFalse(filtered[4, 8])
        self.assertTrue(removed[4, 8])


if __name__ == "__main__":
    unittest.main()
