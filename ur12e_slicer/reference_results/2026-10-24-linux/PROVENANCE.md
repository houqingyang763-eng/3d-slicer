# Linux 部署软件验收：2026-10-24

本目录归档完整通过轮次 `20261024T090044Z_4e8a81`，MoveIt 子轮次为 `20261024_170101_892aad`。入口为 [index.json](index.json)、[本轮检查](validation/summary.json)、[环境与范围](deployment_context.json)。

实际环境为当前电脑的 **WSL2 + WSLg、Ubuntu 24.04、普通用户、独立 Linux 目录中的显式白名单源码快照**，运行真正的 Linux Slicer 5.12.4。该快照不是 git clone，真实 Git checkout 属于另一项验证。未复制 Windows state、colcon build 或既有模型目录；UR 模型从固定上游 ZIP 在线恢复。ROS apt 包此前已经安装，Slicer 从经过 SHA256/SHA512 校验的缓存归档恢复。这是部署软件验收，不代表另一台全新实体 Linux 电脑已经验证。

- 几何 26、桥协议 10、资源 4、指标/gate 10 项检查通过。
- 离线 72 项场景通过预期判据；完整表见 [offline/report.html](offline/report.html)。
- 实际 MoveIt/mock 执行 3 项，执行前失效拒绝 6 项，见 [moveit_mock/report.html](moveit_mock/report.html)。
- Linux GUI 显示、独立 Slicer 重开 MRB 检查通过；附一张真实 Linux GUI 头部截图。
- 已记录进程组全部清理。停止阶段仍出现已知 MoveIt `-11`，**不能写成全部组件正常退出**。

CSV、HTML、truth 及无路径字段的记录逐字节复制。JSON/小型日志将原 Linux 仓库前缀转为仓库相对路径；summary 的报告/日志链接指向本目录内对应文件。数值、时间、请求 ID、独立真值哈希不变。`provenance.json` 记录每份原位置、原始/发布 SHA256 和变更字段位置；原始文件保留在验证目录。截图来自当前电脑对实际 Linux Slicer GUI 的截图副本，不是 Windows Slicer 场景渲染。

本包不含 MRB、安装软件、缓存归档或各场景重复轨迹。光学观测和目标为模拟，机械臂为 UR mock。本轮没有物理设备精度、临床或 ASTM 符合性结果。

## 补充部署检查

[supplemental_checks/summary.json](supplemental_checks/summary.json) 汇总随后完成的检查：

- root 使用默认在线安装入口，在已具备 ROS apt 依赖的环境中幂等重跑成功，未启动应用；保留经过路径规范的小型安装日志。这没有证明另一台新系统冷安装。
- 普通用户离线轮次 `20261024T090333Z_940920` 通过，并包含新增的 12 项生命周期回归和 4 项资源检查；保留该轮 summary、小日志、离线 HTML/CSV。
- 实际启动两次复用同一 backend/Slicer PID，再停止；最终 status 两者均不运行。保留两次启动、stop 小日志和最终状态。

这些补充记录同样保留原始/发布 SHA256。安装/启停文件来自验证 checkout 的上级目录，路径前缀规范为 `validation_checkout_parent/`。停止边界仍保留：MoveIt 曾报 `-11`，进程组已清理不等于全部组件正常退出。
