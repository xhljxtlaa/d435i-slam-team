#!/usr/bin/env bash
set -euo pipefail
if pgrep -x vins_node >/dev/null || pgrep -x orb3_node >/dev/null || pgrep -x realsense2_camera_node >/dev/null || pgrep -x realsense-viewer >/dev/null; then
  echo '先停止估计器、相机驱动和 Viewer；IMU 标定需要独占设备。'
  exit 1
fi
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HERE/results"
cd "$HERE/results"
# Install matching pyrealsense2 separately; no local .so or virtualenv is committed.
exec python3 "$HERE/rs-imu-calibration.py" "$@"
