> 整理说明（2026-11-16）：本文保留历史排查过程。当前入口是 `../start.sh`，采用 depth_imu.yaml；下方“当前纯双目”描述属于此前排查阶段。最新状态以仓库 docs/STATUS.md 为准。

# D435i 标定与累积点云

**最新运行模式已改为相机测距深度 + 左红外跟踪 + IMU 融合。**
配置为 `config/d435i/depth_imu.yaml`；相机配置已开启深度输出。
实现、启动及验证见 [DEPTH_IMU.md](DEPTH_IMU.md)。
下方纯双目记录仅为此前排查历史，不代表当前运行状态。

当前设备序列号：261722075550；SDK / Python 绑定：2.58.4。

## 标定状态

当前 VINS 已应用 2026-11-06 的棋盘格双目标定：8×6 内角点，方格边长
24.4 mm，48 对 640×480 同步图像；双目 RMS 0.130 px，留出样本 RMS
0.125 px，基线 49.910 mm。原始图像、角点及报告位于
checkerboard/session_20261106_1045。旧配置备份在 config/d435i/backup_*。
IMU 到左相机仍使用 SDK 外参初值；右相机外参由新的双目相对外参推导。
放射状点云排查后，当前关闭外参在线优化（estimate_extrinsic: 0），
并保留 stereo_imu_online_before_radial_fix.yaml 作为此前配置备份。
固定外参后的首个 15 秒观察中三轴位置跨度均低于约 1.1 mm；
该对照受场景及人工运动影响，不能替代完整运动精度验证。
这不等于完成相机—IMU 联合标定。
驱动报告 IMU Calibration is not available，尚未进行六姿态标定。
diagnostics.json 和 imu_diagnostic.csv 是运行中诊断，采样时相机有运动，
不能将其均值当作静态偏置，也不能将其中标准差直接作为 Allan 噪声参数。

## 启动当前标定后的配置

### 当前静止漂移排查结果

2026-11-06，用户确认相机静止后，固定外参的双目惯性配置依然发散：
15 秒三轴位置跨度约 2248 / 513 / 3454 米，轨迹无效。
相同硬件与标定下，纯双目对照 30 秒三轴位置跨度均小于 0.03 mm。
这是估计器输出的静止波动，不是实际定位精度证明。
后续 60 秒观察出现数米位置变化，但没有同步图像运动记录，不能认定为
严格静止测试。再次同步记录 45 秒后，图像 5 Hz 光流中位数 0.009 px，
但峰值 115 px，说明该段画面出现明显变化；对应位姿跨度约
54 / 23 / 46 mm。因此还不能宣称纯双目长期稳定，也不能仅凭短时对照
认定唯一故障在 IMU。静态验证必须同时保持相机与场景不动。
静止 IMU 平均加速度模长约 9.882 m/s²，平均角速度约
[-0.00462,-0.00107,-0.00639] rad/s；时间戳连续且 IMU 约 200 Hz。
上述结果将问题集中到惯性融合链路，但不能单独区分 IMU 内参、相机—IMU
外参、时间偏移、初始化及实现问题，也不能判断硬件损坏。

**当前运行纯双目诊断配置 stereo_only_diagnostic.yaml，IMU 未参与估计。**
IMU 六姿态标定及动态融合验证通过之前，不应将此状态称为双目惯性已修复。

分别在不同终端运行：
```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py config_file:=/home/he/vins_ws/config/d435i/camera.yaml
```
```bash
source /opt/ros/humble/setup.bash
source /home/he/vins_ws/install/setup.bash
ros2 run vins vins_node /home/he/vins_ws/config/d435i/stereo_imu.yaml
```

上方是双目惯性配置，仍需排查。若要复现当前纯双目对照，将最后的配置路径
替换为 /home/he/vins_ws/config/d435i/stereo_only_diagnostic.yaml。

camera.yaml 以驱动支持的 config_file 方式关闭红外投射器，避免之前未声明的
launch 参数未生效；不要再单独传入 depth_module.emitter_enabled launch 参数。

## 官方六姿态 IMU 标定

工具来自 librealsense v2.58.4 tools/rs-imu-calibration；未修改上游工具。
Python 依赖安装在当前 calibration/python 目录，不影响系统 Python。

1. 停止 VINS、ROS 相机和 RealSense Viewer，准备稳定的支撑面。
2. 在交互终端运行 `bash /home/he/vins_ws/calibration/run_imu_calibration.sh`。
3. 根据英文提示依次保持六个方向，每个方向稳定后自动采集。
   工具以镜头朝向、底部螺孔的方向描述姿态；按工具提示操作。
4. 提示保存原始数据时输入有意义的后缀，例如日期与设备序列号。
   工具会生成原始加速度/角速度数据、calibration.json、calibration.bin。
5. 检查结果后，在写入相机提示处决定是否写入。写入会更改设备持久标定。
   仅采集/计算时回答 N；本次尚未采集六姿态数据、也未写入任何标定。
6. 写入成功后重新连接相机，再检查驱动警告和静态数据，重新启动 VINS。

这一步校准 IMU 内参/偏置；联合标定需要尺寸已知的棋盘格或 AprilGrid、
双目图像和六轴运动数据。不能用静止数据估计完整相机—IMU 外参及时间偏移。
标定板尺寸应实际测量，采集需覆盖视野及不同距离、充分激励三个旋转轴。

参考：https://github.com/realsenseai/librealsense/tree/v2.58.4/tools/rs-imu-calibration
参考：https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration

## 累积地图

运行（另开终端）：
```bash
source /opt/ros/humble/setup.bash
python3 /home/he/vins_ws/tools/accumulate_cloud.py
```

输入 /point_cloud 是 VINS 已变换到 world 的稀疏点，无需再次应用当前相机位姿。
输出 /accumulated_cloud 是 PointCloud2，1 Hz 发布，持久 QoS。
默认 5 cm 体素，最多 200000 点；超过容量淘汰最久未更新的体素。
仅接受距同时间附近的 VINS 位姿 0.25–8 m 的点，且同一体素需在至少三帧
出现；单帧同体素重复点不重复计数。缺少时间相近的位姿时不累积。
距离过滤只用于显示地图，不修改 VINS 求解器，也不能替代漂移修正。
重新初始化或开始新一轮运行后应清空地图。/vins_restart 为 true 时会自动清空。
时间戳倒退也会清空。内部重定位等其他重置场景应手动清空。

```bash
ros2 service call /vins_cloud_accumulator/clear std_srvs/srv/Trigger '{}'
ros2 service call /vins_cloud_accumulator/save std_srvs/srv/Trigger '{}'
ros2 service call /vins_cloud_accumulator/stats std_srvs/srv/Trigger '{}'
```

保存到 /home/he/vins_ws/output/accumulated_cloud.ply。
RViz 配置：/home/he/vins_ws/config/d435i/vins.rviz，显示项 Accumulated Map。
这是稀疏点云累积；不是深度稠密建图，也不会修正轨迹漂移或回环后的历史点。

累积节点默认启用 clear_on_rviz_start，检测到新的 RViz 进程时自动清空地图，
然后重新累积；通过 /rosout 发布端的 DDS GID 区分重启前后的同名 RViz。
需要保留地图时可启动累积节点并传入 --ros-args -p clear_on_rviz_start:=false。
统一的重启入口还会在打开窗口前主动清空地图：
```bash
bash /home/he/vins_ws/tools/restart_rviz.sh
```
