#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$WS/logs" "$WS/vendor" "$WS/deps"
touch "$WS/vendor/COLCON_IGNORE" "$WS/deps/COLCON_IGNORE"
exec > >(tee -a "$WS/logs/deploy.log") 2>&1
trap 'echo "部署失败，行号 $LINENO。完整日志：$WS/logs/deploy.log"' ERR
JOBS="${ORB3_BUILD_JOBS:-2}"
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo 'ORB3_BUILD_JOBS 必须为正整数'; exit 1; }
for tool in git cmake g++ colcon pkg-config python3; do
  command -v "$tool" >/dev/null || { echo "缺少命令：$tool"; exit 1; }
done
for package in libeigen3-dev libboost-serialization-dev libssl-dev libglew-dev libgl1-mesa-dev libx11-dev libjpeg-dev libpng-dev libopencv-dev; do
  [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null)" == 'install ok installed' ]] || {
    echo "缺少系统依赖 $package；安装后重新执行本脚本。"; exit 1;
  }
done
set +u
source /opt/ros/humble/setup.bash
set -u
for package in cv_bridge message_filters rclcpp; do ros2 pkg prefix "$package" >/dev/null; done
export CMAKE_PREFIX_PATH="$WS/deps:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$WS/deps/lib:${LD_LIBRARY_PATH:-}"

# Do not update existing checkouts or erase local changes on subsequent runs.
if [[ ! -d "$WS/vendor/Pangolin/.git" ]]; then
  git init "$WS/vendor/Pangolin"
  git -C "$WS/vendor/Pangolin" remote add origin https://github.com/stevenlovegrove/Pangolin.git
  git -C "$WS/vendor/Pangolin" fetch --depth 1 origin dd801d244db3a8e27b7fe8020cd751404aa818fd
  git -C "$WS/vendor/Pangolin" checkout --detach FETCH_HEAD
  git -C "$WS/vendor/Pangolin" submodule update --init --recursive
fi
cmake -S "$WS/vendor/Pangolin" -B "$WS/vendor/Pangolin/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$WS/deps" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF \
  -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_VIDEO=OFF
cmake --build "$WS/vendor/Pangolin/build" --parallel "$JOBS"
cmake --install "$WS/vendor/Pangolin/build"

if [[ ! -d "$WS/vendor/ORB_SLAM3/.git" ]]; then
  git init "$WS/vendor/ORB_SLAM3"
  git -C "$WS/vendor/ORB_SLAM3" remote add origin https://github.com/UZ-SLAMLab/ORB_SLAM3.git
  git -C "$WS/vendor/ORB_SLAM3" fetch --depth 1 origin 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
  git -C "$WS/vendor/ORB_SLAM3" checkout --detach FETCH_HEAD
fi
export ORB3_SOURCE_DIR="$WS/vendor/ORB_SLAM3"
python3 - <<'PY'
import os
from pathlib import Path
p = Path(os.environ['ORB3_SOURCE_DIR']) / 'CMakeLists.txt'
s = p.read_text()
# Match Pangolin v0.6's C++14 interface; keep the upstream source otherwise.
p.write_text(s.replace('-std=c++11', '-std=c++14'))
# Upstream's status getter reads LocalMapping's current keyframe even before
# inertial initialization, when that pointer has not necessarily been assigned.
p = Path(os.environ['ORB3_SOURCE_DIR']) / 'src/System.cc'
s = p.read_text()
old = 'double System::GetTimeFromIMUInit()\n{\n    double aux ='
new = ('double System::GetTimeFromIMUInit()\n{\n'
       '    if (!mpAtlas->isImuInitialized()) return 0.0;\n'
       '    double aux =')
if old in s:
    p.write_text(s.replace(old, new, 1))
elif new not in s:
    raise RuntimeError('Unexpected System.cc status getter; review the upstream change')
PY
for component in DBoW2 g2o Sophus; do
  cmake -S "$WS/vendor/ORB_SLAM3/Thirdparty/$component" \
    -B "$WS/vendor/ORB_SLAM3/Thirdparty/$component/build" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=14 -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF
  cmake --build "$WS/vendor/ORB_SLAM3/Thirdparty/$component/build" --parallel "$JOBS"
done
if [[ ! -s "$WS/vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt" ]]; then
  tar -xf "$WS/vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz" -C "$WS/vendor/ORB_SLAM3/Vocabulary"
fi
cmake -S "$WS/vendor/ORB_SLAM3" -B "$WS/vendor/ORB_SLAM3/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$WS/vendor/ORB_SLAM3/build" --target ORB_SLAM3 --parallel "$JOBS"
cd "$WS"
colcon build --packages-select d435i_orb3 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DORB_SLAM3_ROOT="$WS/vendor/ORB_SLAM3" \
  -DPython3_EXECUTABLE=/usr/bin/python3
{
  date --iso-8601=seconds
  git -C "$WS/vendor/Pangolin" rev-parse HEAD
  git -C "$WS/vendor/ORB_SLAM3" rev-parse HEAD
  pkg-config --modversion opencv4
} > "$WS/logs/build_versions.txt"
git -C "$WS/vendor/ORB_SLAM3" diff > "$WS/logs/orb_local_changes.patch"
touch "$WS/.build-complete"
echo "编译完成。启动 RGB-D：$WS/start.sh rgbd；启用 IMU：$WS/start.sh rgbd_imu"
