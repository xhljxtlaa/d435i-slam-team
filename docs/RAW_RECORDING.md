# 原始录制与后续回放

截至 2026-11-18，没有在原工作区、保存版、相机诊断和下载目录找到 D435i 的 `.bag/.db3/.mcap` 原始录制。当前设备未连接；本次共享的是已有标定观测、IMU 数值采样及估计结果。完整图像＋深度＋IMU 原始回放数据仍待实机录制。

## 采集要求

使用 `scripts/record_bag.sh`，先启动相机驱动。默认录 20 秒，保存红外、深度、IMU、CameraInfo、TF 及匹配的 ORB3/VINS 配置。`ORB3_INTRINSICS=vins` 默认核对本项目设备和内参；换用另一台相机时先完成其标定，或显式选择 `ORB3_INTRINSICS=driver` 仅对照驱动参数。

录制器使用 sqlite3 和 zstd 消息压缩、256 MiB 分片；需要 Humble 的 rosbag2 与 zstd 压缩插件。相机连接和投影检查失败会停止；它不会自动启用内置摄像头或生成合成数据。

先录静置片段检查数据链路，再录有固定纹理、平移和多轴转动的片段，并在场景说明里记录实际操作。需要有参考测量才能讨论动态精度。没有运动的样本不用于宣称惯性初始化稳定。

采集成功后检查 `recording.json` 和 `ros2 bag info`。topic 数量非零只是最低条件；还要检查相邻帧时间差、频率、深度单位、零深度占比、CameraInfo、TF 和算法运行结果。保存原始时间戳，不把 CSV 位姿输出重新包装成传感器真值。

## 后续回放流程（待真实数据验收）

在独立 ROS 域中回放，关闭真实相机驱动和其他估计器。各终端均执行 `export ROS_DOMAIN_ID=74`，再 source `/opt/ros/humble/setup.bash`。不要同时启动两套算法；对比时分别播放同一份 bag。

VINS：先 source `vins_ws/install/setup.bash`。把录制会话 `config/vins/` 的三个标定/算法 YAML 复制到新的结果目录，修改 `depth_imu.yaml` 的 `output_path` 和 `pose_graph_save_path` 指向本轮结果目录。启动：

```bash
ros2 run vins vins_node /本轮结果目录/depth_imu.yaml --ros-args -p use_sim_time:=true
```

ORB3：先 source `orb3_ws/install/setup.bash`，并按 `orb3_ws/start.sh` 设置 `LD_LIBRARY_PATH`。使用这次录制配套的 `config/orb3_camera.yaml`，直接启动节点；原 `start.sh` 会重新采集实时设备配置，因此不作为离线回放入口：

```bash
ros2 run d435i_orb3 orb3_node --ros-args \
  -p mode:=rgbd -p settings:=/录制会话/config/orb3_camera.yaml \
  -p vocabulary:=/仓库/orb3_ws/vendor/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  -p output_dir:=/本轮结果目录 -p use_sim_time:=true -p viewer:=false \
  -p dense_mapping:=false
```

估计器就绪后，另一终端回放：

```bash
ros2 bag play /录制会话/bag --clock --delay 3
```

第一轮建议检查 RGB-D；惯性模式仍需满足初始化条件与相机—IMU 标定要求。记录重置次数和轨迹段号。这里提供的是操作流程，当前没有真实 bag，尚未完成上述回放的实机验收。

## 共享方式

为录制会话生成逐文件 SHA256，保留 `recording.json`、全部分片、`metadata.yaml` 和匹配配置。先完整回放验证，再打包上传为新的 GitHub Release 附件，并更新 `data/README.md` 与状态字段。不要只上传某一个 `.db3` 分片。
