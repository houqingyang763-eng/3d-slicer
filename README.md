# 3D Slicer 影像导航与 UR12e 仿真

本仓库包含基础配准练习，以及 UR12e、ROS 2 / MoveIt 2、3D Slicer 的仿真联动和导航误差评价。当前交付以 **Ubuntu 24.04、x86_64 桌面系统**为部署目标。更新日期：2026-10-24。

**接手的 Codex 先读 [AGENTS.md](AGENTS.md)，操作者按 [Linux 部署与验收](ur12e_slicer/DEPLOYMENT_LINUX.md)执行。** 安装、启动、运行和检查均使用仓库入口，不需要原作者的用户名、Windows 路径、WSL 镜像或 Codex 私有运行时。

## 本周交付

1. Slicer 目标设置 → 文件桥 → MoveIt 规划与 UR12e mock 执行 → 关节、工具反馈。
2. 公共 MR 头部影像、非单位坐标变换、合成配准点与独立检查点、模拟光学观测。
3. 离线误差与失效场景；三个影像目标的 MoveIt/mock 执行和六类执行前拒绝检查。
4. 参考平台调研、跨平台评价协议和可读的 JSON / CSV / HTML 结果。

这些成果验证软件、几何和仿真链路。光学观测与真值是合成的；尚未连接物理定位仪或真实机械臂，没有完成实物精度测试或 ASTM 标准符合性测试。

## Linux 快速入口

当前文件树已移除早期 Windows 软件和工作缓存，旧提交历史仍然较大；少量早期练习场景保留 LFS 指针。浅克隆并跳过当前 Linux 任务不需要的旧 LFS 文件自动下载：

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 https://github.com/houqingyang763-eng/3d-slicer.git
cd 3d-slicer
bash ur12e_slicer/bootstrap/install_linux.sh
bash ur12e_slicer/start_linux.sh
bash ur12e_slicer/run_navigation_linux.sh
```

`install_linux.sh` 需要联网和安装系统包时的 `sudo` 权限；后续启动、运行、停止使用普通用户。每一步都应退出成功再继续。完整依赖、Slicer 路径覆盖、后台模式、预期结果、日志和排错见 [部署说明](ur12e_slicer/DEPLOYMENT_LINUX.md)。

完成后：

```bash
bash ur12e_slicer/stop_linux.sh
```

只检查离线几何时，在已安装 Python 3 和 NumPy 的环境运行：

```bash
python3 ur12e_slicer/navigation_benchmark/run_offline.py \
  --output "$PWD/ur12e_slicer/output/my_offline_run"
```

## 先看哪些文件

| 入口 | 用途 |
|---|---|
| [Linux 部署与验收](ur12e_slicer/DEPLOYMENT_LINUX.md) | 环境、安装、启动、结果检查、故障处理 |
| [Codex 交接约定](AGENTS.md) | 工作目录、命令、证据边界和维护规则 |
| [目标、已知与任务清单](ur12e_slicer/README.md) | 阶段目标与已完成 / 待完成内容 |
| [本轮导航成果](ur12e_slicer/NAVIGATION_BENCHMARK_RESULTS_2026-10-23.md) | 2026-10-23 参考运行、数据与限制 |
| [历史参考结果与来源](ur12e_slicer/reference_results/2026-10-23/PROVENANCE.md) | 已发布的离线 / MoveIt 报告与原始记录，不等于接收方本机通过 |
| [本次 Linux 部署验证](ur12e_slicer/reference_results/2026-10-24-linux/PROVENANCE.md) | WSL2 / WSLg、普通用户、独立 Linux 目录中的完整实跑与已知限制 |
| [导航几何与离线评价](ur12e_slicer/navigation_benchmark/README.md) | 坐标约定、固定场景、指标与单元测试 |
| [平台评价协议](ur12e_slicer/research/BENCHMARK_PROTOCOL_v1.md) | 可比条件、静态定位与完整导航的评价范围 |
| [baseline 调研](ur12e_slicer/research/BASELINE_REVIEW_2026-10-23.md) | 平台与公开数据的选择理由 |
| [文件通信协议](ur12e_slicer/protocol.md) | 请求、状态、坐标与误差字段 |

## 目录

| 位置 | 内容 |
|---|---|
| `ur12e_slicer/bootstrap/` | Linux 环境、资源恢复、mock 后台生命周期 |
| `ur12e_slicer/ros2_ws/src/ur12e_slicer_bridge/` | ROS 2 Python 包和固定 mock launch |
| `ur12e_slicer/slicer/` | Slicer 面板、任务文件桥、头部影像结果查看器 |
| `ur12e_slicer/navigation_benchmark/` | 不依赖 ROS 的坐标链、统计、执行前检查和离线试验 |
| `ur12e_slicer/validation/` | ROS 实际执行检查、场景核查、环境与结果采集 |
| `ur12e_slicer/research/` | 研究资料、来源审计、评价协议 |
| `data/` | 公开示例影像和来源记录 |
| `docs/`、`scripts/` | 较早的 Slicer 入门、配准练习和 Windows 工具 |
| `ur12e_slicer/output/`、`ur12e_slicer/runtime/` | 当前机器的新结果、日志和进程状态；不能当作克隆后已运行的证据 |

构建产物、安装包、下载缓存和进程状态不随源码交付；必需资源由安装步骤恢复。参考结果与本机新结果分开解释，详见部署说明。

## Windows 旧入口

已有 Windows + WSL 用户可继续参考 [使用与验收](ur12e_slicer/使用与验收.md)、[环境记录](docs/环境与验证记录.md)和 `Start_UR12e_Demo.cmd` / `Stop_UR12e_Demo.cmd`。这些入口依赖此前本机部署的 Windows Slicer 与专用 WSL 发行版，不是 Linux 部署步骤。当前提交不再携带 Windows Slicer；新 Windows 环境需自行准备，不保证旧启动器可直接运行。Linux 不执行 `.cmd` / `.ps1`。

早期练习见 [模拟点配准实践](docs/模拟点配准实践_2026-10-17.md)、[项目入门说明](docs/项目入门说明.md)和 [Slicer 基础操作](docs/Slicer基础操作.md)。早期 MRI 与 CT 是独立公开示例，没有在仓库中实现影像融合。
