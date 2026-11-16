# D435i ORB-SLAM3 / ROS 2 Humble

本工作区使用原生编译：官方 ORB-SLAM3 核心、Pangolin v0.6，以及本地
`d435i_orb3` ROS 2 接口。所有源码、编译结果和依赖均位于本工作区。
相机配置复制自已跑通的 VINS 工作区；不修改 VINS 文件。

当前状态：GitHub 与 USB 访问已恢复，核心库和 ROS 2 接口已完整编译。
RGB-D 实机运行 50 秒，正常退出并保存 1128 个相机位姿；其中 30 秒
审计采集 756 个位姿，约 25.3 Hz，全部为正常跟踪状态、有限数值和
严格递增时间戳。惯性模式运行 80 秒，输出 1868 个位姿、未崩溃，
55 秒审计观察到初始化完成；但完整运行发生 14 次运动不足引发的地图
重置，因此尚未通过持续稳定运行验证。详见 `logs/imu_validation_final_audit.json`
与 `logs/imu_validation_final.log`。跨地图重置的位姿范围不能作为漂移指标。
`.build-complete` 表示编译成功，不代表已经完成动态精度或联合标定验证。

## 编译

在具有网络访问能力的本机终端运行：

```bash
/home/he/orb3_ws/tools/deploy.sh
```

脚本复用系统已有依赖，Pangolin 安装到 `deps/`，无需 sudo。
g2o 与 ORB 核心统一使用 C++14 和 native SIMD 编译；ROS 2 节点使用
C++17，通过独立后端交换普通数值，避免跨编译单元的 Eigen 分配/释放
不一致。还为上游 `GetTimeFromIMUInit()` 增加初始化前保护，避免状态
查询过早访问当前关键帧。这些设置已用实机运行及退出保存验证。
默认双线程编译，避免 16 GB 内存机器同时编译过多文件。
可用 `ORB3_BUILD_JOBS=1 tools/deploy.sh` 进一步降低内存峰值。
首次下载确定的源码 SHA、OpenCV 版本及核心库的 C++14 补丁保存在 `logs/`；
重复执行会复用已有源码，不自动升级或重置本地修改。

源码：
- https://github.com/UZ-SLAMLab/ORB_SLAM3
- https://github.com/stevenlovegrove/Pangolin/tree/v0.6

## 启动

相机已启动时复用现有驱动。需要独立启动时，在终端一运行：

```bash
/home/he/orb3_ws/start_camera.sh
```

终端二先验证 RGB-D：

```bash
/home/he/orb3_ws/start.sh rgbd
```

停止 RGB-D 节点后启用惯性模式：

```bash
/home/he/orb3_ws/start.sh rgbd_imu
```

启动程序会采集实时 CameraInfo、深度格式、IMU 时间戳和 TF。
只有深度与左红外投影/分辨率/光学坐标一致时才生成配置。
配置包含相机坐标到实际 IMU 消息坐标系的 `T_b_c1`；IMU 噪声暂用初值，
这些参数不代表已经完成联合标定。两种模式均保留当前相机 IMU 数据流。
初始化时让相机看到固定纹理，惯性模式需缓慢平移和多轴转动。
目前建议使用已通过验证的 `rgbd` 模式。惯性模式需继续完成运动初始化
及相机—IMU 标定；初始化状态短暂为 true 不代表后续优化已稳定。

## 复用上次 VINS 相机标定

当前默认读取 `config/vins_intrinsics.json`，复用 2026-11-06 的左目棋盘格
标定，而非驱动的零畸变内参。标定是在同一个 `infra1/image_rect_raw`
话题上采集的，640×480；其小幅非零畸变描述这些已校正像素的残余误差。
不额外整幅去畸变或移动深度像素；ORB 在提取深度时使用原像素，在投影时
使用校正后的特征坐标。配置采集会核对 SDK 设备序列号、分辨率和驱动投影。

参数与原始报告的快照在 `config/calibration/`，详细核对结果见
`config/CALIBRATION_REUSE.md`。只复用左目内参与畸变；IMU 外参仍来自实时
TF 的 SDK 初值，上次并未完成相机—IMU 联合标定或 IMU 噪声标定。

如需对照驱动参数：

```bash
ORB3_INTRINSICS=driver /home/he/orb3_ws/start.sh rgbd
```

相机设备、分辨率、裁剪或校正过程改变时，需要重新核对相应标定配置。

输入：
- `/camera/camera/infra1/image_rect_raw`：mono8，640×480，30 FPS。
- `/camera/camera/depth/image_rect_raw`：16UC1 毫米，或 32FC1 米。
- `/camera/camera/imu`：加速度 m/s²，角速度 rad/s。

图像订阅使用 Reliable、Volatile QoS，匹配本机驱动的可靠发布端；
IMU 使用 SensorData QoS。本机对照测试中，Best Effort 图像订阅接收的
深度约 10 Hz，Reliable 约 25–28 Hz。红外/深度时间差不超过 5 ms。
IMU 时间戳必须严格递增，每帧等待 IMU 覆盖到图像时间并插值终点；
所有缓存有长度上限。跟踪状态为 OK 时才发布位姿。

## 输出与验证

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /orb3/pose
ros2 topic echo /orb3/pose --once
/home/he/orb3_ws/view.sh
```

`view.sh` 自动配置 RViz Fixed Frame 为 `orb3_map`，显示 Path 和 Pose。
默认关闭 Pangolin 窗口；需要官方地图/特征窗口时可以运行
`ORB3_VIEWER=true /home/he/orb3_ws/start.sh rgbd`。
ROS 接口输出位姿、轨迹、姿态式里程计和 ORB 稀疏特征地图点。
当前默认关闭深度表面地图的生成、缓存和融合，只显示特征点。
`/orb3/odometry` 未估计速度或协方差，不能直接按完整惯性导航输出使用。
点云地图使用与位姿相同的 `orb3_map` 坐标系；不依赖额外 TF。
可选表面模式按关键帧保存深度点，并使用优化后的关键帧位姿重新融合。
点云话题、保存与清空操作见 [POINTCLOUD.md](POINTCLOUD.md)。
`/orb3/tracking_state` 中 2 表示正常跟踪；`/orb3/imu_initialized` 为
true 才表示惯性初始化已完成。RGB-D 模式中该值始终为 false。
地图重置、完全丢失或 IMU 初始化改变世界坐标系时，会清空显示轨迹，
并增加 `/orb3/trajectory_segment` 段号，避免把不同坐标系的位置连线。
短暂跟踪丢失后在同一地图重定位，仍可保留原轨迹。此分段用于正确显示，
不代表已经校正惯性误差、完成联合标定或验证了重定位精度。

每次启动生成 `logs/run_日期时间_随机后缀/`：
- `camera.yaml` 与 `.json`：本次实时几何、深度比例与实测 IMU 频率。
- `camera_pose.tum`：运行中发布的相机位姿，米、qx qy qz qw。
  重置/惯性世界对齐时插入 `# segment ...` 注释；需按段评估，不能忽略
  段边界后把整份文件当作单一连续坐标系轨迹。
- `optimized_trajectory.tum`、`keyframes.tum`：正常退出且退出时跟踪正常，
  由 ORB 导出的结果。初始化失败或正在重置时跳过导出，避免上游空地图访问。
- `map.ply`、`map_sparse.ply`：最近一次表面地图和稀疏地图快照，手动保存
  或正常退出时生成；地图重置前的快照保存为 `map_epoch_N.ply`。

回环校正可能改变世界坐标中的历史位姿；实时 Path 是历史发布记录，
退出后导出的优化轨迹用于离线评估。失效后重新建图也可能建立新坐标系，
不能把跨重置的实时记录视为单一连续参考系中的轨迹。

建议录制相同数据分别测试 VINS 与 ORB，检查静置漂移、往返尺度、回环和
丢失后的恢复。相机—IMU 联合标定和动态精度验证仍需完成。

## 当前可离线执行的检查

```bash
for script in tools/deploy.sh start.sh start_camera.sh; do bash -n "$script"; done
g++ -std=c++14 -Wall -Wextra -pedantic tools/test_imu_buffer.cpp -o /tmp/orb3_imu_test
/tmp/orb3_imu_test
python3 -m py_compile tools/capture_config.py
```

这些检查不替代完整核心库/ROS 接口编译与真实相机运行验证。
