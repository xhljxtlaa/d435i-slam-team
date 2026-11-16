# 部署与复现

基线为 Ubuntu 22.04 / ROS 2 Humble，RealSense SDK 2.58.4，OpenCV 4.5.4。`dependencies.lock.json` 是本机实际源码提交。两个工作区独立构建，使用新终端启动不同算法，避免环境叠加。

## 系统依赖

先安装 ROS 2 Humble 与匹配的 RealSense ROS2 驱动；已有驱动可先复用。下面列出本项目使用的常见开发依赖，由操作者在本机终端安装：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git pkg-config python3-colcon-common-extensions \
  python3-numpy python3-opencv libopencv-dev libeigen3-dev \
  libboost-filesystem-dev libboost-system-dev libboost-program-options-dev \
  libboost-serialization-dev libssl-dev libglew-dev libgl1-mesa-dev libx11-dev \
  libjpeg-dev libpng-dev libcurl4-openssl-dev libgoogle-glog-dev libgflags-dev \
  libsuitesparse-dev libatlas-base-dev \
  ros-humble-cv-bridge ros-humble-message-filters ros-humble-image-transport \
  ros-humble-sensor-msgs-py ros-humble-rviz2
```

没有 RealSense ROS2 驱动时，可安装适用的 `ros-humble-realsense2-camera` 和 `ros-humble-realsense2-description`，或按组内锁定版本构建。系统包随软件源变化，安装结果必须记录；安装到了不同 SDK/驱动版本不算重现了原环境。

VINS 本机使用 Ceres 2.1.0。若已有兼容版本，可通过 `Ceres_DIR=/实际路径/lib/cmake/Ceres ./vins_ws/build.sh` 指定；原电脑的 `$HOME/opt/ceres-2.1.0` 会被自动发现。新电脑可运行：

```bash
./scripts/build_ceres.sh
./vins_ws/build.sh
```

`build_ceres.sh` 将锁定源码和安装放在仓库 `.deps/`，不更改系统 Ceres。完整干净机器的依赖安装与构建仍需组员验收；本次整理只执行 CHECKS 中列出的检查。

## D435i 设备检查

关闭 RealSense Viewer 后再启 ROS 相机，避免重复占用设备。`lsusb -t` 应显示相机位于 USB 3.x 链路（如 5000M）。此前本机权限问题通过上游 udev 规则解决：

```bash
sudo install -m 644 system/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

随后拔插相机。相机配置使用 `config_file:=...`；当前启用左红外、深度、加速度计与陀螺仪，`unite_imu_method: 2`，关闭右红外和彩色，空间滤波开启、视差滤波关闭。算法中的 RGB-D 接口实际输入是左红外图和深度。

启动相机后在另一个终端检查：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/imu
```

目标配置：图像/深度 640×480、30 FPS，IMU 约 200 Hz；实际接收频率受 QoS 和负载影响。深度为 16UC1 毫米或经验证的 32FC1 米。必须核对图像域、光学坐标系和时间同步，不能仅凭话题存在判断正常。

## 标定复用和工具

本机标定属于序列号 `261722075550` 的设备与既定成像链，不能直接套到另一台 D435i。ORB3 默认读取 `config/vins_intrinsics.json`，会核对序列号、分辨率和投影；可用 `ORB3_INTRINSICS=driver ./orb3_ws/start.sh rgbd` 对照驱动内参。

棋盘采集脚本位于 `vins_ws/calibration/capture_checkerboard.py`。重新采集双目时需要另启双目相机配置（当前深度模式关闭右目），然后运行：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=false enable_depth:=false enable_infra1:=true enable_infra2:=true \
  depth_module.infra_profile:=640x480x30
```

```bash
source /opt/ros/humble/setup.bash
python3 vins_ws/calibration/capture_checkerboard.py \
  --cols 8 --rows 6 --square-mm 24.4 --output /你的新会话目录
python3 vins_ws/calibration/solve_stereo.py /你的新会话目录
```

先查看报告，再决定是否执行 `--apply`。该工具只自动替换 `left.yaml/right.yaml/stereo_imu.yaml`，切换当前 `depth_imu.yaml` 时还需审核其 `body_T_cam1` 与新标定的关系；ORB3 的标定快照及 profile 也需显式同步，不能只替换一份参数就视为全项目更新。

六姿态工具：先安装与 SDK 匹配的 Python `pyrealsense2`（本机版本 2.58.4），停止设备占用程序，运行 `vins_ws/calibration/run_imu_calibration.sh`。结果写入忽略的 `calibration/results/`。上游工具可能询问是否写入设备持久标定，操作者在核对结果后决定；本次整理没有运行或写入标定。

## 运行与输出

按照根 README 运行，每次只有一套相机驱动与目标估计器。ORB3 每次启动从实时 CameraInfo/TF 生成参数，并将结果写入 `orb3_ws/logs/run_*/`；VINS 写入 `vins_ws/output/run_*/`。历史配置里的绝对路径只作归档；新 VINS 启动器会生成可运行的路径副本。

VINS 累积节点输出 `vins_ws/output/accumulated_cloud.ply`，可用其 clear/save/stats 服务管理。RViz 重启清空地图功能沿用原工具；`restart_rviz.sh` 会关闭本机已有 RViz 窗口，使用前先保存其他会话。

## 验证

```bash
./scripts/check.sh
```

此命令只检查语法、路径迁移和既有 C++ 回归用例，不接相机。`vins_ws/tests/recovery_integration.py` 是合成输入测试，只允许 `ROS_DOMAIN_ID=73`；运行前在独立终端 source 此仓库 VINS 工作区并确认该域没有实际相机。实际硬件验证按 BACKLOG 和实验模板执行。
