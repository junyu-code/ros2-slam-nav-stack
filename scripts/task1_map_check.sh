#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

MAP_NAME="${1:-nav_test_map}"
MAP_DIR="src/slam_nav_bringup/map"
MAP_YAML="${MAP_DIR}/${MAP_NAME}.yaml"
STATIC_WORLD="src/slam_nav_simulation/world/nav_test_world/nav_test_world.world"
DYNAMIC_WORLD="src/slam_nav_simulation/world/nav_test_world/nav_test_world_dynamic.world"

python3 - "$MAP_YAML" "$STATIC_WORLD" "$DYNAMIC_WORLD" <<'PY'
from __future__ import annotations

import ast
import collections
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path

yaml_path = Path(sys.argv[1])
static_world = Path(sys.argv[2])
dynamic_world = Path(sys.argv[3])

errors = 0
warnings = 0


def ok(message: str) -> None:
    print(f"[task1-map-check] OK: {message}")


def warn(message: str) -> None:
    global warnings
    warnings += 1
    print(f"[task1-map-check] WARN: {message}", file=sys.stderr)


def fail(message: str) -> None:
    global errors
    errors += 1
    print(f"[task1-map-check] FAIL: {message}", file=sys.stderr)


def mtime(path: Path) -> str:
    if not path.exists():
        return "missing"
    return datetime.fromtimestamp(path.stat().st_mtime).strftime("%Y-%m-%d %H:%M:%S")


def has_uncommitted_changes(path: Path) -> bool:
    # 只在 world 内容确实偏离 Git 基线时提示重建图；单纯 touch 不再误报。
    try:
        result = subprocess.run(
            ["git", "diff", "--quiet", "--", str(path)],
            cwd=Path.cwd(),
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:  # noqa: BLE001
        return True
    return result.returncode != 0


def parse_value(raw: str):
    value = raw.strip()
    if not value:
        return ""
    if value.startswith("[") and value.endswith("]"):
        return ast.literal_eval(value)
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value.strip("'\"")


def parse_simple_yaml(path: Path) -> dict[str, object]:
    data: dict[str, object] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, raw_value = stripped.split(":", 1)
        data[key.strip()] = parse_value(raw_value)
    return data


def read_pgm(path: Path) -> tuple[str, int, int, int, bytes]:
    raw = path.read_bytes()
    idx = 0
    tokens: list[bytes] = []
    while len(tokens) < 4:
        while idx < len(raw) and raw[idx] in b" \t\r\n":
            idx += 1
        if idx < len(raw) and raw[idx] == ord("#"):
            while idx < len(raw) and raw[idx] not in b"\r\n":
                idx += 1
            continue
        start = idx
        while idx < len(raw) and raw[idx] not in b" \t\r\n":
            idx += 1
        if start == idx:
            break
        tokens.append(raw[start:idx])
    while idx < len(raw) and raw[idx] in b" \t\r\n":
        idx += 1
    if len(tokens) != 4:
        raise ValueError("PGM header 不完整")
    magic = tokens[0].decode("ascii")
    width = int(tokens[1])
    height = int(tokens[2])
    maxval = int(tokens[3])
    return magic, width, height, maxval, raw[idx:]


print(f"[task1-map-check] 地图 yaml: {yaml_path}")
if not yaml_path.exists():
    fail(f"地图 yaml 不存在: {yaml_path}")
    print(f"[task1-map-check] 汇总: errors={errors}, warnings={warnings}")
    sys.exit(1)

try:
    metadata = parse_simple_yaml(yaml_path)
except Exception as exc:  # noqa: BLE001
    fail(f"无法解析 yaml: {exc}")
    print(f"[task1-map-check] 汇总: errors={errors}, warnings={warnings}")
    sys.exit(1)

image_value = metadata.get("image")
if not isinstance(image_value, str) or not image_value:
    fail("yaml 缺少 image 字段")
    pgm_path = yaml_path.with_suffix(".pgm")
else:
    pgm_path = (yaml_path.parent / image_value).resolve()
    ok(f"yaml image -> {pgm_path}")

resolution = metadata.get("resolution")
origin = metadata.get("origin")
mode = metadata.get("mode", "unset")
occupied_thresh = metadata.get("occupied_thresh", "unset")
free_thresh = metadata.get("free_thresh", "unset")
negate = metadata.get("negate", "unset")

if not isinstance(resolution, (int, float)) or float(resolution) <= 0:
    fail(f"resolution 异常: {resolution}")
    resolution = 0.0
else:
    ok(f"resolution={float(resolution):.4f} m/pixel")

if not isinstance(origin, list) or len(origin) < 3:
    fail(f"origin 异常: {origin}")
    origin = [0.0, 0.0, 0.0]
else:
    ok(f"origin=[{float(origin[0]):.3f}, {float(origin[1]):.3f}, {float(origin[2]):.3f}]")

if not pgm_path.exists():
    fail(f"地图 pgm 不存在: {pgm_path}")
    print(f"[task1-map-check] 汇总: errors={errors}, warnings={warnings}")
    sys.exit(1)

try:
    magic, width, height, maxval, pixels = read_pgm(pgm_path)
except Exception as exc:  # noqa: BLE001
    fail(f"无法解析 PGM: {exc}")
    print(f"[task1-map-check] 汇总: errors={errors}, warnings={warnings}")
    sys.exit(1)

if magic != "P5":
    fail(f"PGM magic 应为 P5，实际为 {magic}")
else:
    ok("PGM 格式为 P5 raw greymap")

expected_bytes = width * height * (1 if maxval < 256 else 2)
if len(pixels) < expected_bytes:
    fail(f"PGM 像素数据不足: expected>={expected_bytes}, actual={len(pixels)}")
else:
    ok(f"PGM 尺寸={width} x {height}, maxval={maxval}")

extent_x = width * float(resolution)
extent_y = height * float(resolution)
ok(f"地图覆盖范围约 {extent_x:.2f} m x {extent_y:.2f} m")
ok(f"yaml 大小={yaml_path.stat().st_size} bytes, pgm 大小={pgm_path.stat().st_size} bytes")
ok(f"yaml 更新时间={mtime(yaml_path)}, pgm 更新时间={mtime(pgm_path)}")
print(f"[task1-map-check] 阈值: mode={mode}, occupied_thresh={occupied_thresh}, free_thresh={free_thresh}, negate={negate}")

if maxval < 256 and pixels:
    counter = collections.Counter(pixels[:expected_bytes])
    occupied = counter.get(0, 0)
    unknown = counter.get(205, 0)
    free = counter.get(254, 0) + counter.get(255, 0)
    total = max(1, expected_bytes)
    print(
        "[task1-map-check] 像素统计: "
        f"occupied={occupied} ({occupied / total:.1%}), "
        f"free={free} ({free / total:.1%}), "
        f"unknown={unknown} ({unknown / total:.1%})"
    )

for world in (static_world,):
    if world.exists() and world.stat().st_mtime > min(yaml_path.stat().st_mtime, pgm_path.stat().st_mtime):
        if has_uncommitted_changes(world):
            warn(f"static world content changed after map was saved: {world}; rebuild/save {yaml_path.name} before final acceptance")
        else:
            ok(f"world mtime is newer but content matches git baseline: {world}")

print("[task1-map-check] 可转写到 EXPERIMENT_RECORD.md 的地图结果:")
print(f"  - resolution: {float(resolution):.4f} m/pixel")
print(f"  - origin: [{float(origin[0]):.3f}, {float(origin[1]):.3f}, {float(origin[2]):.3f}]")
print(f"  - image size: {width} x {height} pixels")
print(f"  - map extent: {extent_x:.2f} m x {extent_y:.2f} m")
pgm_display_path = Path(os.path.relpath(pgm_path, Path.cwd()))
print(f"  - files: {yaml_path} ({yaml_path.stat().st_size} bytes), {pgm_display_path} ({pgm_path.stat().st_size} bytes)")
print(f"[task1-map-check] 汇总: errors={errors}, warnings={warnings}")
sys.exit(1 if errors else 0)
PY
