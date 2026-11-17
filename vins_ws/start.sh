#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
[[ -f "$WS/install/setup.bash" ]] || { echo "先运行 $WS/build.sh"; exit 1; }
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
mkdir -p "$WS/output"
RUN_DIR="$(mktemp -d "$WS/output/run_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
CONFIG="$(python3 "$WS/tools/prepare_runtime.py" --run-dir "$RUN_DIR")"
echo "本轮输出：$RUN_DIR"
exec ros2 run vins vins_node "$CONFIG" "$@"
