#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# task1 验收主线使用静态场地，避免动态障碍物混入建图和静态避障成功率统计。
exec "${SCRIPT_DIR}/start_simulation.sh" world:=static "$@"
