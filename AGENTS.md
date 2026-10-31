# Codex 接手指南

## 目标与部署入口

当前主任务为 `ur12e_slicer/`：复现 UR12e + ROS 2 / MoveIt 2 + 3D Slicer 链路，以及头部影像 / 模拟光学的导航评价。默认 mock 路线保留；2026-10-27 新增固定本机 Docker 端点的 URSim 路线。先读 [DEPLOYMENT_LINUX.md](ur12e_slicer/DEPLOYMENT_LINUX.md)，URSim 另读 [URSIM_LINUX.md](ur12e_slicer/URSIM_LINUX.md)，再执行命令。本文是给接收方 Codex 的工作约定。

最新进展入口为 [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md)。10 月 27 日 URSim 全通过是历史记录；10 月 31 日两轮基础 5/5、Slicer 20/21（录帧 19<20），外层失败。头模恢复是沿用模拟坐标关系的会话显示，未重跑完整 URSim 头部导航评价。精选证据见 `ur12e_slicer/reference_results/2026-10-31-ursim/`。

- 首个支持目标：Ubuntu 24.04（Noble）、x86_64 / amd64 桌面系统。
- 系统 Python 3.12 + apt NumPy / PyYAML；ROS 2 Jazzy、MoveIt 2、UR 官方驱动；Slicer 5.12.4 使用自己的 Python。
- 默认 mock 路线无需 Docker、Gazebo 或 URSim；可选 URSim 路线增加本机 Docker。两条 Linux 路线均无需 Codex 私有 Python / Node、Anaconda、Windows 软件或 WSL 镜像。
- 安装系统包时需要 `sudo`；启动、构建和实验使用普通用户，不能把工作区固定为 `/root`。
- 当前提交不携带 Windows 程序 / 工作缓存；旧 Git 历史仍含这些大文件，早期练习保留少量 LFS 指针。新获取使用 `GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 ...`，不要为当前任务拉全量历史或执行全量 `git lfs pull`。

## 目录与运行约定

| 目录 / 文件 | 职责 |
|---|---|
| `bootstrap/install_linux.sh` | 安装与依赖 / 资源准备；失败应先读本轮日志 |
| `start_linux.sh`、`run_navigation_linux.sh`、`stop_linux.sh` | Linux 启动、完整评价、停止入口 |
| `status_linux.sh` | 只读 JSON 状态；不启动 / 停止进程 |
| `bootstrap/install_ursim.sh` | 可选 URSim 的 Docker 依赖和固定资源准备 |
| `start_ursim_linux.sh`、`run_ursim_linux.sh`、`stop_ursim_linux.sh` | 独立 URSim 启动、基础控制验收和停止入口 |
| `status_ursim_linux.sh` | 只读 URSim 容器、GUI relay、ROS / Slicer 状态 |
| `ros2_ws/src/ur12e_slicer_bridge/` | ROS 节点、协议工具、固定 mock / 本机 URSim launch 与机器人侧配置 |
| `slicer/` | Slicer 面板、文件桥和导航场景 |
| `navigation_benchmark/` | 纯 NumPy 几何、真值 / 估计分离、指标和授权检查 |
| `validation/` | ROS 闭环、静态指标、场景与环境检查 |
| `research/` | 文献来源、baseline 审计与 benchmark 协议 |
| `reference_results/2026-10-23/` | 已发布的历史参考证据与来源；不是本机验证输出 |
| `output/` | 本机运行结果及安装 / 启动日志 |
| `runtime/` | 本机临时通信、PID / 心跳和锁；不提交到 Git |

mock 控制文件仍在项目 `runtime/`。URSim 高频 `state.json`、`target.json`、`validation.lock`、`ros_events.jsonl` 改放当前用户原生 Linux cache 的 `ur12e_slicer/<项目路径哈希>/ursim/bridge/`；基础验收原始结果放同级 `validation/`。项目 `runtime/` 继续保存所有权、Slicer 心跳和任务桥队列。查询 `status_ursim_linux.sh` 的 `control_runtime` 取得本机实际控制路径，不复制旧项目 `runtime/state.json` 作为 URSim 状态。完成或失败的基础验收均逐字节归档到项目 `output/ursim_validation/` 并核对 SHA256。

以上路径相对 `ur12e_slicer/`。从仓库根目录运行公开命令，始终引用带空格的路径。`SLICER_EXECUTABLE` 可显式指定本机 Linux Slicer 可执行文件，默认 `ur12e_slicer/.local/slicer/Slicer`；`UR12E_BUILD_DIR` 可改为普通用户可写的构建目录，不要指向另一任务的 colcon 工作区。默认构建目录位于 `${XDG_CACHE_HOME:-$HOME/.cache}/ur12e_slicer/<项目路径哈希>/ros2_ws`。

## 必须保留的运行边界

1. mock 入口继续固定 `ur_type=ur12e`、`use_mock_hardware=true`、`robot_ip=127.0.0.1`。URSim 入口固定 `ur_type=ur12e`、`use_mock_hardware=false` 和本任务 Docker 端点 `192.168.62.2`，只接受本机 Unix Docker socket、固定镜像 digest、项目标签和独立内部桥网络。此处 `false` 表示连接虚拟控制柜，不授权连接实物；两条入口均不开放任意真机 IP。不要为排错修改这些边界。
2. ROS 使用本任务 `ROS_DOMAIN_ID=62`、`ROS_LOCALHOST_ONLY=1`。不要接入实验室既有机器人网络，也不要停止其他任务的 ROS / Slicer 进程。
3. mock 与 URSim 共享 ROS domain 和前端管理，不同时运行；它们使用各自的控制文件目录。运行评价时，Slicer 面板暂停手工发送目标；同一控制目录只允许一个发送端。停止必须调用对应任务入口，PID、容器 / 网络身份不符或仍有进程时应保留记录并报错。
4. 读到 `ready=true` 还应核对反馈新鲜度、对应的 `source=ros2_mock_hardware` 或 `ros2_ursim` 和真实 ROS 服务 / action。URSim 还要核对虚拟控制柜、External Control 程序与 controller 状态。前端动画、旧文件和纯协议测试不能当作成功执行。
5. 安装或网络失败后停止依赖步骤，保留完整退出码与日志，解决原因后重跑。不要临时禁用 TLS / 哈希校验、伪造依赖成功、放宽测试阈值或删掉失败记录。

## 坐标、真值与评价规则

- 通用桥协议：UR `base` 下米、四元数 `xyzw`、误差弧度；Slicer / 导航几何：mm。`T_A_B` 把 B 中的点映射到 A。只在明示接口换算单位；`base` 和 `base_link` 不等同。
- 机器人端简化针尖默认是 `tool0` 局部 +Z 120 mm。规划位置约束与误差都对应针尖，不是法兰原点。集中参数见 `ros2_ws/src/ur12e_slicer_bridge/config/bridge.yaml`。
- 拟合点、检查点、控制使用的估计标定、评分使用的真值必须分开。保留冻结真值和哈希。合成真值不能称为物理独立测量。
- 无效 / 遮挡 / 过期观测应拒绝，不补零位姿或零误差。`NavigationGate` 只实现执行前应用层检查；直接通用桥命令可绕过它，不能声称已实现硬件安全停机。
- 当前 MR 是公共示例，配准点和目标为合成几何。MR 等值面未经解剖分割审核，也没有作为 MoveIt 头模碰撞模型。
- 所有误差注入是确定性软件检查。当前没有真实定位仪 / 机械臂联调、实物精度、临床验证或 ASTM 合规结果。SlicerROS2 为软件参照，未安装 / 复现成当前桥。

## 修改后怎样检查

至少先运行与修改相关的离线检查；启动 / 协议 / 几何变更还要通过对应真实运行。详见部署说明的逐级验收。

```bash
python3 -m unittest discover -s ur12e_slicer/navigation_benchmark -p 'test_*.py' -v
python3 ur12e_slicer/validation/check_navigation_metrics.py
PYTHONPATH=ur12e_slicer/ros2_ws/src/ur12e_slicer_bridge \
  python3 -m unittest discover \
  -s ur12e_slicer/ros2_ws/src/ur12e_slicer_bridge/tests -v
```

离线检查不证明 ROS 规划执行。完整运行使用 `bash ur12e_slicer/run_navigation_linux.sh`，检查 `output/linux_validation/<本轮标识>/summary.json` 的 `passed`，以及导航联动结果中三个执行场景、六个拒绝场景与日志，并确认停止无残留。`--offline-only` 不启动 ROS / Slicer；`--no-slicer` 另跑 ROS mock，但跳过 GUI；`--stop-after` 在运行后清理本任务进程。不要把参考结果文件复制为本机通过；不要为了重现历史末位数值更改判据。

URSim 基础控制使用 `bash ur12e_slicer/run_ursim_linux.sh`，检查本轮 `output/ursim_validation/<标识>/lifecycle_summary.json`、其引用的基础控制 `summary.json` 和原始 action / joint / 文件桥记录。不带 `--no-slicer` 时还要求本轮 Slicer 目标 / 规划 / 执行 / 反馈验收完成，并记录 `slicer_passed` 及 `slicer_result`；`--no-slicer` 不验证 GUI。`--stop-after` 在结束时停止本任务组件。URSim 通过也不等于旧 mock 的整套头模导航评价已在 URSim 重跑。首次只配虚拟示教器时使用 `start_ursim_linux.sh --simulator-only`，该模式不启动 ROS / Slicer、不发送运动。

## 交付与维护

- 修改入口 / 参数后同步根 README、部署说明与相关子目录 README。
- Git 提交源码、配置、环境 / 数据来源、必要小型示例和精选结果；不提交构建缓存、安装包、进程文件、个人绝对路径、整套软件或论文采集缓存。
- `python3 tools/prepare_ur12e_release.py` 检查发布白名单；`--copy-to <新的空目录>` 可准备独立源码副本，运行后写 `package_inventory.json`。白名单为 `ur12e_slicer/publish_manifest.json`；它不执行 Git 提交 / 推送。
- 报告必须分开写：当前文件检查、本次实跑、历史参考运行、尚未验证事项。Windows + WSL 证据不能写成原生 Linux 全新机器验证。
- 当前已知退出异常、无硬件和标准范围等限制应保留；“任务进程已结束”不等于“全部组件无错误退出”。
