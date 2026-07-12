#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


OCCUPIED = np.uint8(0)
FREE = np.uint8(254)


@dataclass(frozen=True)
class MapThresholds:
    occupied: float
    free: float
    negate: bool


@dataclass(frozen=True)
class FilterStats:
    input_occupied: int
    input_free: int
    input_unknown: int
    removed_obstacle_cells: int
    filled_hole_cells: int
    closing_added_cells: int
    free_opening_removed_cells: int
    output_occupied: int
    changed_cells: int


def classify_map(
    image: np.ndarray,
    thresholds: MapThresholds,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    shade = image.astype(np.float32) / 255.0
    occupancy = shade if thresholds.negate else 1.0 - shade
    occupied = occupancy > thresholds.occupied
    free = occupancy < thresholds.free
    unknown = ~(occupied | free)
    return occupied, free, unknown


def remove_small_obstacle_groups(
    occupied: np.ndarray,
    minimum_size: int,
) -> tuple[np.ndarray, np.ndarray]:
    if minimum_size <= 1:
        return occupied.copy(), np.zeros_like(occupied)

    result = occupied.copy()
    removed = np.zeros_like(occupied)
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        occupied.astype(np.uint8),
        connectivity=8,
    )
    for label in range(1, component_count):
        if stats[label, cv2.CC_STAT_AREA] < minimum_size:
            component = labels == label
            result[component] = False
            removed[component] = True
    return result, removed


def fill_small_free_holes(
    occupied: np.ndarray,
    unknown: np.ndarray,
    maximum_size: int,
) -> tuple[np.ndarray, np.ndarray]:
    if maximum_size <= 0:
        return occupied.copy(), np.zeros_like(occupied)

    result = occupied.copy()
    filled = np.zeros_like(occupied)
    background = (~occupied).astype(np.uint8)
    component_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        background,
        connectivity=8,
    )
    rows, columns = occupied.shape
    for label in range(1, component_count):
        x, y, width, height, area = stats[label]
        touches_border = (
            x == 0 or y == 0 or x + width == columns or y + height == rows
        )
        if touches_border or area > maximum_size:
            continue
        component = labels == label
        if np.any(unknown[component]):
            continue
        result[component] = True
        filled[component] = True
    return result, filled


def close_obstacle_mask(
    occupied: np.ndarray,
    unknown: np.ndarray,
    radius_cells: int,
) -> tuple[np.ndarray, np.ndarray]:
    if radius_cells <= 0:
        return occupied.copy(), np.zeros_like(occupied)

    size = radius_cells * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (size, size))
    result = cv2.morphologyEx(
        occupied.astype(np.uint8),
        cv2.MORPH_CLOSE,
        kernel,
    ).astype(bool)
    result |= occupied
    result[unknown] = occupied[unknown]
    added = result & ~occupied
    return result, added


def open_free_space_mask(
    free: np.ndarray,
    radius_cells: int,
) -> tuple[np.ndarray, np.ndarray]:
    if radius_cells <= 0:
        return free.copy(), np.zeros_like(free)

    size = radius_cells * 2 + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (size, size))
    result = cv2.morphologyEx(
        free.astype(np.uint8),
        cv2.MORPH_OPEN,
        kernel,
    ).astype(bool)
    removed = free & ~result
    return result, removed


def unknown_pixel_value(thresholds: MapThresholds) -> np.uint8:
    probability = (thresholds.occupied + thresholds.free) / 2.0
    shade = probability if thresholds.negate else 1.0 - probability
    return np.uint8(round(np.clip(shade, 0.0, 1.0) * 255.0))


def encode_map(
    occupied: np.ndarray,
    free: np.ndarray,
    unknown: np.ndarray,
    thresholds: MapThresholds,
) -> np.ndarray:
    occupied_value = FREE if thresholds.negate else OCCUPIED
    free_value = OCCUPIED if thresholds.negate else FREE
    output = np.full(occupied.shape, unknown_pixel_value(thresholds), np.uint8)
    output[free] = free_value
    output[unknown] = unknown_pixel_value(thresholds)
    output[occupied] = occupied_value
    return output


def filter_map(
    image: np.ndarray,
    thresholds: MapThresholds,
    minimum_obstacle_size: int,
    maximum_hole_size: int,
    closing_radius_cells: int,
    free_opening_radius_cells: int,
) -> tuple[
    np.ndarray,
    FilterStats,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    occupied, free, unknown = classify_map(image, thresholds)
    input_free = free.copy()
    input_unknown = unknown.copy()
    input_free_count = int(np.count_nonzero(free))
    filtered, removed = remove_small_obstacle_groups(
        occupied,
        minimum_obstacle_size,
    )
    free = free | removed

    filtered, filled = fill_small_free_holes(
        filtered,
        unknown,
        maximum_hole_size,
    )
    free[filled] = False

    before_closing = filtered.copy()
    filtered, closing_added = close_obstacle_mask(
        filtered,
        unknown,
        closing_radius_cells,
    )
    free[closing_added] = False

    free, free_opening_removed = open_free_space_mask(
        free,
        free_opening_radius_cells,
    )
    unknown |= free_opening_removed

    output = encode_map(filtered, free, unknown, thresholds)
    changed = (
        (filtered != occupied) |
        (free != input_free) |
        (unknown != input_unknown)
    )
    stats = FilterStats(
        input_occupied=int(np.count_nonzero(occupied)),
        input_free=input_free_count,
        input_unknown=int(np.count_nonzero(unknown)),
        removed_obstacle_cells=int(np.count_nonzero(removed)),
        filled_hole_cells=int(np.count_nonzero(filled)),
        closing_added_cells=int(np.count_nonzero(filtered & ~before_closing)),
        free_opening_removed_cells=int(np.count_nonzero(free_opening_removed)),
        output_occupied=int(np.count_nonzero(filtered)),
        changed_cells=int(np.count_nonzero(changed)),
    )
    return output, stats, occupied, filtered, input_free, free, unknown


def load_metadata(path: Path) -> dict[str, object]:
    with path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError(f"map YAML must contain a mapping: {path}")
    return metadata


def resolve_image_path(yaml_path: Path, metadata: dict[str, object]) -> Path:
    image_value = metadata.get("image")
    if not isinstance(image_value, str) or not image_value:
        raise ValueError(f"map YAML has no image path: {yaml_path}")
    return (yaml_path.parent / image_value).resolve()


def thresholds_from_metadata(metadata: dict[str, object]) -> MapThresholds:
    mode = str(metadata.get("mode", "trinary"))
    if mode != "trinary":
        raise ValueError(f"only trinary maps are supported, got: {mode}")
    thresholds = MapThresholds(
        occupied=float(metadata.get("occupied_thresh", 0.65)),
        free=float(metadata.get("free_thresh", 0.25)),
        negate=bool(int(metadata.get("negate", 0))),
    )
    if not 0.0 <= thresholds.free < thresholds.occupied <= 1.0:
        raise ValueError(
            "map thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1"
        )
    return thresholds


def write_difference_preview(
    path: Path,
    original_occupied: np.ndarray,
    filtered_occupied: np.ndarray,
    original_free: np.ndarray,
    filtered_free: np.ndarray,
    unknown: np.ndarray,
) -> None:
    preview = np.full((*original_occupied.shape, 3), 255, np.uint8)
    preview[unknown] = (160, 160, 160)
    preview[filtered_free] = (255, 255, 255)
    preview[filtered_occupied] = (0, 0, 0)
    preview[original_free & ~filtered_free & ~filtered_occupied] = (0, 165, 255)
    preview[filtered_occupied & ~original_occupied] = (0, 0, 255)
    preview[original_occupied & ~filtered_occupied] = (0, 180, 0)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), preview):
        raise RuntimeError(f"failed to write difference preview: {path}")


def preprocess(args: argparse.Namespace) -> FilterStats:
    input_yaml = args.input_yaml.resolve()
    output_yaml = args.output_yaml.resolve()
    if input_yaml == output_yaml:
        raise ValueError("input and output YAML paths must be different")
    metadata = load_metadata(input_yaml)
    if args.occupied_threshold is not None:
        metadata["occupied_thresh"] = args.occupied_threshold
    if args.free_threshold is not None:
        metadata["free_thresh"] = args.free_threshold
    thresholds = thresholds_from_metadata(metadata)
    input_image_path = resolve_image_path(input_yaml, metadata)
    output_image = output_yaml.with_suffix(".pgm")
    if input_image_path == output_image:
        raise ValueError("input and output image paths must be different")
    image = cv2.imread(str(input_image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"failed to load map image: {input_image_path}")

    (
        output,
        stats,
        original_occupied,
        filtered_occupied,
        original_free,
        filtered_free,
        output_unknown,
    ) = filter_map(
        image,
        thresholds,
        args.minimum_obstacle_size,
        args.maximum_hole_size,
        args.closing_radius_cells,
        args.free_opening_radius_cells,
    )
    changed_ratio = stats.changed_cells / max(1, image.size)
    if changed_ratio > args.maximum_change_ratio:
        raise ValueError(
            f"filter would change {changed_ratio:.2%} of map cells, exceeding "
            f"the {args.maximum_change_ratio:.2%} safety limit"
        )

    output_yaml.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_image), output):
        raise RuntimeError(f"failed to write filtered map: {output_image}")

    output_metadata = dict(metadata)
    output_metadata["image"] = output_image.name
    with output_yaml.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(output_metadata, stream, sort_keys=False, default_flow_style=None)

    difference_preview = (
        args.difference_preview.resolve()
        if args.difference_preview is not None
        else output_yaml.with_name(f"{output_yaml.stem}_diff.png")
    )
    write_difference_preview(
        difference_preview,
        original_occupied,
        filtered_occupied,
        original_free,
        filtered_free,
        output_unknown,
    )

    print(f"[preprocess-map] input:  {input_yaml}")
    print(f"[preprocess-map] output: {output_yaml}")
    print(
        "[preprocess-map] cells: "
        f"occupied {stats.input_occupied} -> {stats.output_occupied}, "
        f"unknown={stats.input_unknown}, changed={stats.changed_cells} "
        f"({changed_ratio:.2%})"
    )
    print(
        "[preprocess-map] operations: "
        f"removed_obstacles={stats.removed_obstacle_cells}, "
        f"filled_holes={stats.filled_hole_cells}, "
        f"closing_added={stats.closing_added_cells}, "
        f"free_opening_removed={stats.free_opening_removed_cells}"
    )
    print(f"[preprocess-map] diff:   {difference_preview}")
    return stats


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Conservatively denoise a trinary ROS occupancy-grid map.",
    )
    parser.add_argument(
        "input_yaml",
        nargs="?",
        type=Path,
        default=Path("src/slam_nav_bringup/map/large_arena.yaml"),
    )
    parser.add_argument(
        "output_yaml",
        nargs="?",
        type=Path,
        default=Path("src/slam_nav_bringup/map/large_arena_filtered.yaml"),
    )
    parser.add_argument("--minimum-obstacle-size", type=int, default=2)
    parser.add_argument("--maximum-hole-size", type=int, default=12)
    parser.add_argument("--closing-radius-cells", type=int, default=1)
    parser.add_argument("--free-opening-radius-cells", type=int, default=0)
    parser.add_argument("--maximum-change-ratio", type=float, default=0.05)
    parser.add_argument("--occupied-threshold", type=float, default=None)
    parser.add_argument("--free-threshold", type=float, default=None)
    parser.add_argument(
        "--difference-preview",
        type=Path,
        default=None,
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    preprocess(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
