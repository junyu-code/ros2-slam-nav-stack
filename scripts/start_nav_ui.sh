#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 兼容旧入口；所有参数和环境变量由统一启动器处理。
exec "${SCRIPT_DIR}/start_ui.sh" "$@"
