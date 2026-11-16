#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-rgbd}"
if [[ $# -gt 0 ]]; then shift; fi
VIEWER="${ORB3_VIEWER:-false}"
INTRINSICS="${ORB3_INTRINSICS:-vins}"
PROFILE_ARGS=()
case "$INTRINSICS" in
  vins) PROFILE_ARGS=(--intrinsics-profile "$WS/config/vins_intrinsics.json") ;;
  driver) ;;
  *) echo 'ORB3_INTRINSICS 必须为 vins 或 driver'; exit 1 ;;
esac
[[ "$MODE" == rgbd || "$MODE" == rgbd_imu ]] || { echo '用法：start.sh [rgbd|rgbd_imu]'; exit 1; }
[[ "$VIEWER" == true || "$VIEWER" == false ]] || { echo 'ORB3_VIEWER 必须为 true 或 false'; exit 1; }
[[ -f "$WS/.build-complete" ]] || { echo "尚未编译，请运行：$WS/tools/deploy.sh"; exit 1; }
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export LD_LIBRARY_PATH="$WS/deps/lib:$WS/vendor/ORB_SLAM3/lib:$WS/vendor/ORB_SLAM3/Thirdparty/DBoW2/lib:$WS/vendor/ORB_SLAM3/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"
RUN_DIR="$(mktemp -d "$WS/logs/run_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
/usr/bin/python3 "$WS/tools/capture_config.py" --output "$RUN_DIR/camera.yaml" "${PROFILE_ARGS[@]}"
echo "输出目录：$RUN_DIR"
exec ros2 run d435i_orb3 orb3_node --ros-args \
  -p mode:="$MODE" -p settings:="$RUN_DIR/camera.yaml" \
  -p vocabulary:="$WS/vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  -p output_dir:="$RUN_DIR" -p viewer:="$VIEWER" \
  -p dense_mapping:=false -p map_voxel_size:=0.05 -p map_pixel_stride:=8 -p map_max_points:=100000 "$@"
