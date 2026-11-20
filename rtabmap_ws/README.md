# D435i 的 RTAB-Map 建图

该流程使用 D435i 的 640×480、15 FPS 彩色图及对齐深度。RTAB-Map 的 RGB-D 里程计负责跟踪，SLAM 节点负责回环检测和位姿图优化。相机内参取自驱动发布的 `CameraInfo`。

## 安装和启动

在 Ubuntu 22.04 / ROS 2 Humble 上先安装相机驱动、RTAB-Map 和 RViz。

```bash
sudo apt install ros-humble-realsense2-camera ros-humble-rtabmap-launch ros-humble-rviz2
```

通过 USB 3 连接 D435i，关闭正在占用相机的其他驱动。然后从仓库根目录运行启动脚本。

```bash
./rtabmap_ws/start.sh
```

RViz 显示实时彩色图、对齐深度、彩色点云和优化轨迹。RTAB-Map 原生窗口显示特征匹配、位姿图和回环。仅开 RViz 可传 `show_viewer:=false show_rviz:=true`。无桌面时可传 `show_viewer:=false`。

若复用已运行的相机驱动，可传 `start_camera:=false`。驱动需提供 `/camera/camera/color/image_raw`、`/camera/camera/aligned_depth_to_color/image_raw`、`/camera/camera/color/camera_info` 和 `camera_link` 对应的 TF。

## 保存地图和导出轨迹

每次启动会在 `rtabmap_ws/output/run_*/` 创建一个 `rtabmap.db`，启动日志会输出完整路径。完成采集后按 `Ctrl+C` 正常退出，将地图保存到数据库。用 RTAB-Map 工具导出以米为单位的优化关键帧轨迹。导出的位姿对应 `camera_link`，格式为 `timestamp x y z qx qy qz qw`。

```bash
source rtabmap_ws/setup.bash
rtabmap-export --poses --poses_format 10 --opt 2 /path/to/rtabmap.db
```

如果同一数据库包含多个未通过回环连接的地图段，依据地图 ID 分开评估。实时 `/rtabmap/mapPath` 显示当前位姿图中的关键帧。

## 实机检查

2026-11-20 在 D435i、ROS 2 Humble 和 RTAB-Map ROS 0.23.13 上实测收到彩色图、对齐深度、RGB-D 里程计、优化轨迹和彩色点云。测试中出现过 USB 断连及视觉跟踪丢失。`Odom/ResetCountdown` 设为 `0`，跟踪丢失时轨迹暂停更新。动态精度仍需带参考数据的测试。

```bash
source rtabmap_ws/setup.bash
ros2 topic echo /rtabmap/odom --once --field header
ros2 topic echo /rtabmap/mapPath --once --field header
ros2 topic echo /rtabmap/cloud_map --qos-durability transient_local --once --field header
```

配置参考 [RTAB-Map 官方 D435i 彩色相机示例](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_color.launch.py) 和 [ROS 2 启动文件](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_launch/launch/rtabmap.launch.py)。
