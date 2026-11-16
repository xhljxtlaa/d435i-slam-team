#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
JOBS="${VINS_BUILD_JOBS:-2}"
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo 'VINS_BUILD_JOBS 必须为正整数'; exit 1; }
set +u
source /opt/ros/humble/setup.bash
set -u
ARGS=(-DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3)
if [[ -n "${Ceres_DIR:-}" ]]; then
  ARGS+=("-DCeres_DIR=$Ceres_DIR")
elif [[ -f "$WS/../.deps/ceres/lib/cmake/Ceres/CeresConfig.cmake" ]]; then
  ARGS+=("-DCeres_DIR=$WS/../.deps/ceres/lib/cmake/Ceres")
elif [[ -f "$HOME/opt/ceres-2.1.0/lib/cmake/Ceres/CeresConfig.cmake" ]]; then
  ARGS+=("-DCeres_DIR=$HOME/opt/ceres-2.1.0/lib/cmake/Ceres")
fi
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
cd "$WS"
colcon build --packages-up-to vins --executor sequential --symlink-install --cmake-args "${ARGS[@]}"
