#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u
exec ros2 launch realsense2_camera rs_launch.py config_file:="$WS/config/camera.yaml"
