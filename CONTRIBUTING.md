# 协作约定

`main` 保存小组可复现的版本。每项工作从最新 `main` 开分支，例如 `feat/orb3-loop`、`fix/vins-recovery`、`calib/d435i-imu`。通过 PR 合并，并请另外一名成员评审。若要强制评审，在 GitHub 分支规则中为 main 配置至少 1 个批准；本仓库模板本身不强制执行远端权限规则。

```bash
git switch main
git pull --ff-only
git switch -c feat/your-task
# 修改后检查
./scripts/check.sh
git add 具体文件路径
git commit -m "fix: 描述实际修复的问题"
git push -u origin HEAD
```

代码与配置变更在同一个 PR 说明触发条件、行为变化、验证方式和剩余问题。涉及算法、标定或传感器模式时，标注相机配置、数据来源、提交 SHA、是否启用 IMU、重置次数及坐标系分段。

标定数据按会话目录新增，保留原图、棋盘实际尺寸和报告；校验后再修改活动参数。不要直接用动态 IMU 均值当静态偏置，或用短时标准差替代 Allan 噪声。

rosbag、点云、运行轨迹放在本地或组内数据存储，在实验说明中记录链接和校验和。`build/`、`install/`、`log/`、`logs/`、`output/`、依赖安装目录不提交。`evidence/` 是本次明确整理的既往小型报告；后续仅提交对结论有用的脱敏证据。

团队共享的精选小型结果可保存在 `data/examples/`，此处的 `.tum/.ply` 已设置明确的 Git 忽略例外。完整历史文件与原始录制使用 Release 附件，并更新 `data/index.json` 或 `data/release.json` 的索引与 SHA256。`git pull` 不会自动获取附件，使用 `python3 scripts/fetch_data.py` 下载既有历史数据包。

硬件验收由组员在有设备的电脑执行，报告通过和失败项目。离线 CI 通过不代表惯性稳定或动态精度通过。
