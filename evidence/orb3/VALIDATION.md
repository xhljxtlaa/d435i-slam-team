# 2026-11-13 实机验证

- GitHub 访问成功，ORB 核心提交 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。
- D435i 通过 USB 3.2 识别；RealSense 驱动/运行库 2.58.4。
- 原生核心库、g2o、Pangolin 及本地 ROS 2 包已编译并动态链接成功。
- 实时 CameraInfo 与 TF 验证红外/深度投影和光学坐标一致，深度 16UC1/mm。
- 当前 TF 使用 SDK 默认 IMU 外参；驱动报告没有可用的 IMU 工厂标定。

## RGB-D：通过本轮运行验证

50 秒限时运行正常响应 SIGINT，并保存 1128 个实时相机位姿及优化轨迹。
其中 30 秒审计记录 756 个正常跟踪位姿，25.30 Hz；时间戳严格递增，
所有位置/姿态数值有限。该静置片段位置范围约 0.38/0.29/0.66 mm，
仅描述本轮观测，不能作为绝对精度或动态精度指标。

- `rgbd_validation.log`
- `rgbd_validation_audit.json`
- `run_20261113_104611_vQH3Bo/`

## RGB-D + IMU：接入运行通过，持续稳定性未通过

80 秒限时运行正常退出，输出 1868 个位姿，无图像队列丢帧和拒收 IMU。
55 秒审计记录 11002 条 IMU、1309 个位姿，观察到初始化状态 true。
但运行日志有 14 次 `TRACK: Reset map because local mapper set the bad imu flag`，
之前提示 `Not enough motion for initializing`。因此不能把该段实时位姿
拼成一个统一世界坐标系中的连续轨迹，也不能据此声称惯性精度验证完成。
后续需要有足够平移和多轴转动的初始化过程、联合标定及带参考的动态测试。

- `imu_validation_final.log`
- `imu_validation_final_audit.json`
- `run_20261113_105041_Da6xhG/`

## 已修复并重新验证的问题

- g2o 默认 C++17 与 ORB C++14 的 Eigen 对齐分配/释放不一致：统一第三方
  编译标准，ROS 节点通过不暴露 Eigen 类型的独立 C++14 后端调用核心库。
- 上游初始化状态 getter 在初始化前访问尚未准备的关键帧：增加早期返回保护。
- Best Effort 大图像订阅在本机有明显丢帧：图像采用 Reliable QoS，IMU
  保持 SensorData QoS。

具体源码差异见 `orb_local_changes.patch`；早期失败日志保留用于诊断，
最终判定以以上 `validation` 文件为准。
