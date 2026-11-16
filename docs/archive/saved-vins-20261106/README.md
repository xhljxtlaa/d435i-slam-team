# D435i 深度 + IMU 定位 · 保存版 v1

这是 2026-11-06 保存的独立代码、配置和可执行文件，运行时不依赖原工作区的
编译产物。依赖这台电脑已安装的 ROS 2 Humble、RealSense、OpenCV 等环境。

## 使用

- 启动：双击桌面/应用菜单“启动 D435i 定位”，或运行本目录 start.sh。
- 停止：关闭 RViz，或在启动终端按 Ctrl+C；也可使用“停止 D435i 定位”或 stop.sh。
- 检查安装：运行 check.sh。
- 启动会关闭此前本机 D435i 工作区的相机/估计器/地图/RViz实例，避免重复占用。
- 先接好 USB 3.x 相机，让视野内有固定纹理，初始化时先静置。
- 程序先检查红外图、深度、IMU 数据，再打开定位与显示；拔插后深度流无法恢复时
  会保存并停止，本次日志会明确提示。插稳相机后重新启动即可。

每次启动使用 runs/日期时间/ 独立目录：
- output/pointcloud.ply：退出前保存的累积稀疏地图。
- output/trajectory.tum：退出前最后一段显示轨迹，米、qx qy qz qw，最多3000位姿。
- output/vio.csv：原始全频率位姿日志；原算法时间戳只保留整秒。
- logs/：相机、估计器、预览、地图及RViz日志。
- config/：本次配置副本，不覆盖其他运行。

若系统强制断电/杀死全部进程，退出前的地图/TUM保存不能保证完成；vio.csv是运行中
持续写入的日志。发生跟踪失效时地图会清空并从新原点初始化，TUM只保存最后一段，
不能把跨重置的CSV直接视作一个连续世界坐标轨迹。

## 保存内容

bin/vins_node 是独立复制的可执行文件；source/ 包含完整源码和 changes.patch。
config/ 保存双目标定、固定 IMU 外参初值、深度/相机参数及RViz配置。
tools/ 包含地图累积、近距离伪彩色显示与启动控制。
tests/ 保存几何、跟踪失效及隔离恢复测试。version.json 和 SHA256SUMS 用于核对版本。
完整历史说明见 DEVELOPMENT_NOTES.md 与 DEPTH_IMU.md。

当前模式：相机硬件深度 + 左红外特征跟踪 + IMU；深度0.18–6m，显示0.18–1m，
累计点云0.18–8m、5cm体素、三帧确认，显示2像素小点。
IMU六姿态/相机—IMU联合标定及长期运动精度验证尚未完成。
累计地图是稀疏路标，非稠密表面重建。

## 恢复源码与重新编译

源码不包含 .git 历史，但保留基准提交号和工作区差异。可以复制
source/VINS-Fusion-ROS2-Humble 到新的ROS工作区src中，在本机依赖环境下执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vins --symlink-install --cmake-args -DCeres_DIR=/home/he/opt/ceres-2.1.0/lib/cmake/Ceres
```

运行预编译保存版不需要重新编译。搬到另一台电脑仍需安装兼容依赖和设备udev规则。
