# 上游、许可证与归档范围

| 组件 | 来源 | 本仓库处理 |
| --- | --- | --- |
| VINS ROS2 移植 | https://github.com/fanhong-li/VINS-Fusion-ROS2-Humble | 完整算法源码及本地修改；保留 LICENCE（GPLv3）和各文件声明 |
| 原始 VINS-Fusion | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion | 上游算法归属详见 VINS 源码及 README |
| ORB-SLAM3 | https://github.com/UZ-SLAMLab/ORB_SLAM3 | 核心按锁定 SHA 获取；GPLv3 文本在 licenses/ |
| Pangolin | https://github.com/stevenlovegrove/Pangolin | 按锁定 SHA 获取，原许可证随上游源码保留 |
| Ceres | https://ceres-solver.googlesource.com/ceres-solver | 按本机 2.1.0 基线 SHA 获取，原许可证随上游保留 |
| RealSense IMU 标定及 udev | https://github.com/realsenseai/librealsense | 原工具脚本与规则，保留各自版权声明；SDK 基线 2.58.4 |

`dependencies.lock.json` 保存本机实际使用的提交；`patches/` 为整理前的 tracked 工作树差异。VINS 的两个新增头文件直接保存在源码目录，不包含在原始 git diff 中。

为了控制体积，未复制 VINS 上游宣传图片、global_fusion 车辆模型及 `brief_k10L6.bin` 大词典。当前 `--packages-up-to vins` 构建与 depth_imu 运行不需要它们；如需独立回环模块，可按锁定的 VINS 提交获取 `support_files/brief_k10L6.bin`，并另行验证 loop_fusion 构建和配置。上游 README 中指向未复制宣传图的链接可能缺图。

新增整理与集成代码按根目录 GPLv3 许可证提供，第三方文件继续遵循各自许可证和版权声明。
