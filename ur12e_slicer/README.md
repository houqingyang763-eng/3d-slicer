# UR12e 与 3D Slicer 联动：任务清单

更新日期：2026-10-24。状态：**基础联调完成；平台评价规范、MR 头模＋模拟光学坐标链、离线误差验证与真实 MoveIt/mock 执行已交付，新增 Linux 部署入口与交接文档。真实光学／机械臂精度尚未验证。** 新机器从 [Linux 部署与验收](DEPLOYMENT_LINUX.md)开始；Codex 先读根目录 [AGENTS.md](../AGENTS.md)。本轮研究结果见 [头模导航与评价成果](NAVIGATION_BENCHMARK_RESULTS_2026-10-23.md)。旧的 21／6／9 项只覆盖原 mock 场景，本轮新证据单独记录；Windows 原有使用步骤见 [使用与验收.md](使用与验收.md)。

本目录是本项工作的统一入口。代码、配置、启动脚本、演示和结果记录随任务推进逐步加入；本清单允许修改、拆分和增补。完成项必须附可复现入口或结果证据。

## Linux 接手入口

Ubuntu 24.04、x86_64 桌面版，在仓库根目录依次执行；每一步退出成功再继续：

```bash
bash ur12e_slicer/bootstrap/install_linux.sh
bash ur12e_slicer/start_linux.sh
bash ur12e_slicer/run_navigation_linux.sh
bash ur12e_slicer/stop_linux.sh
```

只跑离线评价用 `run_navigation_linux.sh --offline-only`；没有显示环境但要测试 ROS mock 用 `start_linux.sh --backend-only` 与 `run_navigation_linux.sh --no-slicer`。这些模式保留未验证 Slicer 的边界。状态查询为 `bash ur12e_slicer/status_linux.sh`，不修改进程。完整环境、参数、输出和排错见 [DEPLOYMENT_LINUX.md](DEPLOYMENT_LINUX.md)。

2026-10-24 已在 WSL2 / WSLg 的普通用户独立目录完成真正 Linux Slicer 与 ROS mock 的完整运行，见 [来源与范围](reference_results/2026-10-24-linux/PROVENANCE.md)。ROS apt 环境和已校验 Slicer 下载缓存有复用，未声称另一台全新实体机从零部署已通过；MoveIt 停止阶段 -11 仍保留为限制。

下述 2026-10-21 至 23 的安装路径、Windows / WSL 入口和数值属于历史参考运行；Linux 接收方不应照搬 `/root`、`/mnt/c/Users/HOU` 或 `D:\\CodexEnvs`，也不能把历史结果当本机通过。

## 目标

以 UR12e 为对象，优先采用 ROS 2／MoveIt 2，实现以下递进功能：

1. 在 Slicer 中实时显示机械臂及工具的位置和朝向。
2. 将 Slicer 中定义的目标转换到机器人坐标，使工具自动对准并保持。
3. 规划机械臂运动路径，先完成仿真联动，再开展实物演示。

首轮已确认交付：Slicer 设置目标位置和朝向 → 坐标转换 → MoveIt 2 规划 → 仿真 UR12e 执行 → Slicer 同步显示机械臂和工具反馈，并记录末端位置、朝向误差；附启动入口、演示录像和结果记录。

## 已知

- 【用户转述的团队答复】设备型号为 UR12e，已经到货；只有一台，团队共享使用。
- 【用户转述的团队答复】先在仿真中开发，仿真联动打通后再尝试实物；路径规划先使用常见现成算法。
- 【用户转述的团队答复】技术方向倾向 ROS 2／MoveIt 2；团队未指定具体版本。当前已部署 Ubuntu 24.04.5／ROS 2 Jazzy／MoveIt 2 2.12.4／UR 驱动 3.9.0，使用专用 WSL 发行版 `UR12e-Jazzy`。
- 【用户转述的团队答复】通信控制、坐标转换、光学定位和路径规划没有严格分工，需要协同对接；不能据此认定全部由本模块独立负责。
- 【用户授权】由 Codex 接管实现、调试和任务清单维护，用户以成果验收为主；本轮是开工前最后一轮集中对齐。
- 【用户确认，2026-10-21】首轮采用简化工具与静态目标；允许在当前电脑建立 Linux 开发环境；没有固定验收日期。
- 【仓库记录】已有 Windows 版 Slicer 5.12.4、示例影像、模拟工具和模拟点配准。尚未接入真实机械臂或光学设备，参见 [项目入口](../README.md) 和 [既有调研](../docs/项目调研_配准与机器人导航_2026-10-17.md)。
- 【本机初检，2026-10-21，安装前】当前系统为 Windows 11；最初 `wsl.exe --status` 返回“未安装适用于 Linux 的 Windows 子系统”。
- 【本机执行结果，2026-10-21】WSL 3.0.3 安装包 SHA256 核对通过，MSI 安装退出码 0；VirtualMachinePlatform 和 Microsoft-Windows-Subsystem-Linux 启用成功并要求重启。Ubuntu 24.04 官方离线包也已下载并核验 SHA256。
- 【既有官方资料核对】UR 官方 ROS 2 驱动列有 `ur12e`，提供 MoveIt 2 配置，要求 Linux，并提供 mock hardware／URSim 仿真入口；最终选定的软件版本仍须核对其对应支持情况。

## 待完成的

1. **建立环境与 UR12e 仿真。**【已完成首轮验证】
   - 在当前 Windows 电脑建立 Linux 开发环境；优先核对 WSL 2、Ubuntu 24.04 与 ROS 2 Jazzy。
   - 选择并固定 ROS 2、MoveIt 2、UR 驱动及模型版本，核对兼容性与来源。
   - 启动 UR12e 仿真，验证关节反馈、运动规划和仿真执行。
   - 交付：环境说明、版本记录和可重复使用的启动入口。
   - 当前进度：Ubuntu 24.04.5、ROS 2 Jazzy、MoveIt 2 2.12.4、UR 驱动 3.9.0 已安装；专用发行版位于 `D:\CodexEnvs\UR12e-Jazzy`。实际 URDF 为官方 GenericSystem，普通 JTC 激活，真实规划服务与执行 action 已验证。

2. **打通 Slicer 的目标、坐标和反馈。**【首轮仿真完成】
   - 确定 Slicer 与 ROS 2 的连接方式，定义目标、状态和错误信息的接口。
   - 明确坐标系、变换方向、长度单位、工具作用点和朝向定义；首轮使用有记录的模拟变换。
   - 从 Slicer 发送目标位置和朝向，将机器人侧的关节和工具状态回传至 Slicer。
   - 交付：可运行的双向联动、坐标关系说明及核对记录。
   - 当前进度：真实 ROS 关节／TF 驱动 Slicer 官方模型与工具；Slicer 面板发目标、规划预览、缓存执行、反馈与保持的 21 项完整验收通过。最终代码重启后实测位置误差 1.2051 mm、整体朝向误差 0.019561 rad，4.062 秒保持内 20 次不同反馈更新，整段满足 2 mm／0.03 rad。自动化桥隔离在本任务 `runtime/slicer_bridge`。

3. **完成路径规划与首轮验收演示。**【首轮仿真完成；退出异常保留】
   - 先用一种现成规划算法，完成静态目标与简单障碍物场景；按结果决定是否增补算法比较。
   - 演示目标设置、规划预览、仿真执行和反馈显示，记录目标与末端的位置、朝向差值。
   - 检查不可达目标、规划失败和连接中断时的可见反馈。
   - 交付：启动入口、可复现场景、演示录像和简明结果记录；本轮验收线为 2 mm／整体朝向角 0.03 rad。
   - 当前进度：昨日机器人侧 7 项验收通过；今日 Slicer 完整闭环、约 13 秒录像、侧障执行／碰撞目标拒绝／移除恢复、反馈停更／恢复均通过。实际算法已核验为 RRTConnect。退出时 MoveIt 段错误、控制器需要 launch 的 SIGTERM 升级，不能称所有组件正常退出；已核验任务进程组全部结束，保留日志和后续修复项。

4. **研究光学参与的头模导航，再迁移实物。**【评价规范与首版头模／模拟光学仿真完成；实物待开展】
   - 【最新执行结果】已有 MR 三视图与头模场景、非单位坐标链、刚体配准和独立检查点、12 类离线场景；三次真实 MoveIt/mock 目标执行与六类执行前拒绝通过。一键入口、自动报告与本轮发现见 [成果记录](NAVIGATION_BENCHMARK_RESULTS_2026-10-23.md)。
   - 【用户澄清后更新】baseline 先保留。benchmark 的目的为跨平台性能可比性，优先核对 ASTM F2554-22 静态点定位测试规范与 SlicerAstmPhantomTest；LARK／ETH 只承担组件算法对比。头模最终目标、方向及动态运动需另补任务，见 [平台评价补查](research/PLATFORM_BENCHMARK_2026-10-23.md)。
   - 先查同类平台与公开 benchmark：本轮未找到满足完整头模／光学／机械臂与独立原始测量的单一可复跑平台。软件参照选 SlicerROS2；头模光学数据首选候选 LARK（数据许可待明确）；时空标定优先 ETH ASL；PLUS 仅作组件回归。详见 [审计与后续验收](research/BASELINE_REVIEW_2026-10-23.md)。
   - 已取得必要小型数据、包清单和公开评测代码入口，尚未复现论文数值。先核统计／单位／真值与原方法，再选择误差研究问题；不直接采用摘要精度或任意噪声得出本机误差排序。
   - 当前优先研究：完整影像／头模参考架／相机／机器人／工具坐标链；标定误差向目标的传递及独立验证；观测时间／有效性与计划失效规则。
   - 头部影像中的冻结合成目标已通过模拟光学观测与非单位标定链驱动机器人；真值与估计分开保存。该场景不含真实光学、不含头模碰撞模型；动态跟踪／停止、实物精度与标准符合性仍未完成。
   - 已有针尖误差来自同一模拟机器人模型，不能用来表示光学导航或真实标定精度。路径规划算法比较和界面打磨后移；退出问题继续作为工程维护项。
   - 核对实际控制器软件、工具、网络和可用机时；与团队确认各接口提供者。
   - 将模拟坐标关系替换为实际底座、工具与影像／跟踪系统之间的标定结果。
   - 先完成真实状态回传，再按实验安排验证运动和定位误差，增补光学定位对接。
   - 交付：实物运行说明、标定来源和实测记录。仿真结果仅作为仿真联动的证据。

## 本轮对齐结论与实施约定

1. 首轮仿真闭环及启动入口、录像、结果记录已经用户确认。
2. 实际末端工具和具体操作尚未确定，首轮用简化工具、静态目标，目标包含位置和朝向。
3. 已获授权在当前电脑建立 Linux 开发环境；具体软件选型与实现由 Codex 负责。
4. 没有固定截止日期，以可复现、可验收的阶段成果推进。
5. 首轮优先采用 Windows Slicer 与外部 ROS 2 的本机文件桥接，复用当前可运行的 Slicer；ROS 2 负责规划和仿真执行，Slicer 显示机器人侧反馈。此项是 Agent 的实施选择，须通过联调验证。

## 证据与维护

- 2026-10-21：依据用户转述的团队答复、用户自治授权和仓库现状建立清单。当前没有新增联动运行结果。
- 2026-10-21 后续：完成本轮最终对齐、WSL 安装、离线包与 UR 模型准备、前后端初版和可在 Windows 完成的组件检查；完整状态与下一步见 [RESULTS.md](RESULTS.md)。
- 2026-10-21 重启后：环境安装完成、官方 mock／控制器／ROS 图通过核验、机器人侧 7 项验收通过、Slicer 真实反馈显示成功。用户要求暂停关机，保存运行证据并正常停止本任务进程；继续步骤见 [RESUME.md](RESUME.md)。
- 2026-10-22：恢复后完成 Slicer 全闭环、实际录像、静态障碍和反馈停更检查。修正录帧阻塞、过期提示、共享文件写入短重试，以及启动／停止就绪等待；保留未通过轮次与退出异常，详见 [RESULTS.md](RESULTS.md)。
- 2026-10-23：用户指出头模／光学与误差环节缺失。已核对代码和官方资料，明确基础联调的证明边界，完成 [实物前研究与优先级](实物接入前研究_2026-10-23.md)；未将研究设计写成新增实验结果。
- 2026-10-23 后续：依用户要求查同类平台和 benchmark，完成 [首轮选型](research/BASELINE_REVIEW_2026-10-23.md)、平台／数据／SlicerROS2 三份审计及小型公开数据检查。当前是候选筛选和材料核验，未复现算法数值；下一阶段先完成外部基线，再实施本项目头模误差场景。
- 2026-10-23 实施：用户批准评价规范＋头模光学链＋误差验证。已完成 [评价协议](research/BENCHMARK_PROTOCOL_v1.md)、几何／指标代码、72 个离线目标评价、真实 MoveIt/mock 三次执行、六类执行前拒绝和 Slicer MR 三视图。未安装或宣称复现 SlicerROS2／LARK 算法；实际成果与边界见 [本轮记录](NAVIGATION_BENCHMARK_RESULTS_2026-10-23.md)。
- 后续进展按“任务状态、产物路径、验证结果、下一步”更新；方案变更保留简短原因，暂定方案不写成已完成结果。
- 官方入口：[UR ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)、[官方仿真说明](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/usage/simulation.html)。

## 使用入口

- Linux 安装与检查：[DEPLOYMENT_LINUX.md](DEPLOYMENT_LINUX.md)。整体启动 `bash ur12e_slicer/start_linux.sh`，完整验证 `bash ur12e_slicer/run_navigation_linux.sh`，停止 `bash ur12e_slicer/stop_linux.sh`；命令在仓库根目录执行。
- 以下 `.cmd` / `.ps1` 为已有 Windows + WSL 环境兼容入口，不用于 Linux 安装。
- 直接操作与观看成果：[使用与验收.md](使用与验收.md)。后续接续工作见 [RESUME.md](RESUME.md)。
- 启动整体仿真：[Start_UR12e_Demo.cmd](Start_UR12e_Demo.cmd)。等待实际 ROS 就绪与专属 Slicer 面板后才报告成功；已运行时复用当前实例。
- 单独查看前端：[slicer/Open_UR12e_Slicer.cmd](slicer/Open_UR12e_Slicer.cmd)。没有 ROS 反馈时显示等待，不代表仿真已经运行。
- 结束本任务：[Stop_UR12e_Demo.cmd](Stop_UR12e_Demo.cmd)。只通知已校验的 launch，等待该组全部非僵尸进程和专属 Slicer 窗口退出；超时明确失败，不自行强杀其他进程。
