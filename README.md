# D435i · ORB-SLAM3 / VINS-Fusion / RTAB-Map 协作工作区

整理自本机 2026-11-06 的 VINS/D435i 与 2026-11-13 的 ORB3/D435i 部署聊天、实际源码、标定和运行记录。`insfusion` 在这些会话中指 VINS-Fusion。

| 路线 | 当前推荐入口 | 已有验证与限制 |
| --- | --- | --- |
| ORB-SLAM3 | `orb3_ws/start.sh rgbd` | 既往 RGB-D 实机跟踪通过；默认仅显示 ORB 稀疏特征点和轨迹 |
| ORB-SLAM3 惯性 | `orb3_ws/start.sh rgbd_imu` | 接入和初始化已有记录，但多次地图重置，稳定性未通过 |
| VINS-Fusion 本地深度扩展 | `vins_ws/start.sh` | 左红外 + D435i 测距 + IMU；有运行及失效恢复记录，动态精度与联合标定待完成 |
| RTAB-Map | `rtabmap_ws/start.sh` | D435i 彩色图和对齐深度；已验证实机轨迹及彩色点云输出 |

完整状态见 [STATUS](docs/STATUS.md)。这次整理完成的离线检查见 [CHECKS](docs/CHECKS.md)，它与此前实机验证分开记录。

## 组员获取数据

先执行 `git pull --ff-only`。本次已补充 [data/examples](data/examples/) 中的轨迹、稀疏点云和实验预览；原有 48 对棋盘标定图仍在 `vins_ws/calibration/checkerboard/`。

[完整历史实验数据下载](https://github.com/xhljxtlaa/d435i-slam-team/releases/tag/data-20261118)：约 24 MB、217 个原始文件，含 ORB3 多次运行、VINS 输出、标定原图、IMU CSV/NPZ。也可运行：

```bash
python3 scripts/fetch_data.py
```

**目前仍缺少同步红外图＋深度＋IMU 的完整原始 rosbag，无法仅靠这些结果重新运行完整定位链路。** 当前没有连接 D435i；已准备 `scripts/record_bag.sh` 供后续实机补录。已有数据的格式、用途和限制详见 [数据说明](data/README.md)。

## 第一次部署

环境基线：Ubuntu 22.04、ROS 2 Humble、OpenCV 4.5.4、RealSense SDK 2.58.4、D435i USB 3.x。不同系统版本需另行验证。

```bash
git clone https://github.com/xhljxtlaa/d435i-slam-team.git
cd d435i-slam-team
```

依赖与设备权限见 [DEPLOYMENT](docs/DEPLOYMENT.md)。已具备相应依赖时：

```bash
# 两条路线分别构建，不混合 source 两个工作区的 install
./orb3_ws/tools/deploy.sh
./vins_ws/build.sh
```

ORB3：三个终端分别运行（每个终端先进入仓库目录）：

```bash
./orb3_ws/start_camera.sh
./orb3_ws/start.sh rgbd
./orb3_ws/view.sh
```

VINS：先停止 ORB3 和它的相机驱动，再在三个终端分别运行：

```bash
./vins_ws/start_camera.sh
./vins_ws/start.sh
./vins_ws/view.sh
```

另外两个终端启动稀疏地图累积与深度预览：

```bash
source /opt/ros/humble/setup.bash
python3 vins_ws/tools/accumulate_cloud.py
```

```bash
source /opt/ros/humble/setup.bash
python3 vins_ws/tools/depth_preview.py
```

VINS 启动器在 `vins_ws/output/run_*/` 生成本轮配置和输出目录，自动解析当前电脑路径。跟踪失效后会清图重建；累积点是稀疏路标。不要把跨地图重置的轨迹拼成同一个世界坐标系。

RTAB-Map 使用 Humble 软件包。安装依赖后，接入相机，在仓库根目录运行 `./rtabmap_ws/start.sh`。启动参数、可视化和轨迹导出见 [RTAB-Map 运行说明](rtabmap_ws/README.md)。

## 仓库内容

- `orb3_ws/`：本地 ROS 2 接口、独立后端、运行脚本和标定复用配置；核心依赖按已验证 SHA 下载。
- `vins_ws/src/`：可以直接修改的 VINS-Fusion ROS2 源码，包含深度约束、PnP 和跟踪失效恢复扩展。
- `rtabmap_ws/` 含 D435i RGB-D 建图启动文件、RViz 配置和轨迹导出说明。
- `vins_ws/calibration/checkerboard/`：48 对原始棋盘图、观测点及标定报告；24.4 mm 方格、8×6 内角点。
- `docs/history/`：两段部署聊天的用户消息与助手最终答复；历史状态可能已被后续覆盖。
- `evidence/`：既往运行审计和最终验证日志；`docs/archive/` 保存冻结版说明。
- `data/`：可直接使用的精选轨迹/点云、数据清单与完整历史数据的 Release 下载入口。
- `dependencies.lock.json`、`patches/`、`docs/SOURCE_MANIFEST.json`：上游提交、原始改动和归档来源。
- `.github/`：问题/实验模板、PR 模板与不接相机的自动检查。

## 三人协作

按 [CONTRIBUTING](CONTRIBUTING.md) 分支、提交和评审。建议一人维护设备与标定，一人负责 ORB3，一人负责 VINS；每个 PR 由另一人检查。账号确定后可填写 [TEAM](docs/TEAM.md)。任务清单见 [BACKLOG](docs/BACKLOG.md)。

原工作区和冻结可执行版本仍保留在原电脑。编译目录、第三方大词典、临时运行输出和二进制包不纳入 Git；精选实验轨迹/点云明确保存在 `data/examples/`，完整历史数据通过 Release 共享。需要的上游资源获取方式见部署文档。历史文档中的 `/home/he/...` 是当时路径，团队运行优先使用本 README 的相对入口。

源码保留上游 GPL 许可证及各文件原有声明，详见 [THIRD_PARTY](THIRD_PARTY.md)。
