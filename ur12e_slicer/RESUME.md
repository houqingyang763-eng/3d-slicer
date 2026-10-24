# 暂停与继续入口

更新时间：2026-10-23。**评价规范、MR 头模＋模拟光学链和真实 MoveIt/mock 首轮验证完成。** 优先读 [本轮成果](NAVIGATION_BENCHMARK_RESULTS_2026-10-23.md)，历史研究顺序和旧暂停快照保留如下。

## 最新工作方向（优先读）

**最新实施状态覆盖下方“尚未实现”的历史描述。** 用户批准的三步已落实为协议、场景和报告：`Run_Navigation_Benchmark.cmd` 一键整轮已通过；`slicer/Open_Navigation_Benchmark.cmd` 只读查看结果。冻结场景为 `navigation_benchmark/fixtures/head_world_v1.json`；最终集成轮 `output/navigation_benchmark/20261023_164512_b45bf5/`，离线轮 `output/navigation_offline/20261023_164510_ae600c/`。26 数学测试、10 指标／授权测试、15 离线解析检查通过，三次规划执行和六类执行前拒绝通过。

授权时间用接收端可信时钟，保留原反馈/TF采样时刻，整包旧 now_s 不能使其重新有效；固定头模变化阈值 0.5 mm／0.5°、帧龄100 ms／时差30 ms 均是假设工程参数。共享文件可见性仅对存储错误重读≤0.25 s。不要把这些参数当设备或安全指标。

本轮已停止自己启动的 mock 进程组并关闭该前端，另外打开只读结果查看器供验收；最新状态以 `runtime/navigation_benchmark_viewer.json` 和实际进程为准。退出软件异常沿用旧限制，不在本轮宣称已修复。后续不要重做安装或误把查看器当机器人已运行。

接续工作：按 [成果记录的界限与下一步](NAVIGATION_BENCHMARK_RESULTS_2026-10-23.md)补目标／姿态覆盖、保持窗口与动态实测设计，并落实实际光学／参考架／工具及独立计量。SlicerROS2 为软件参照，LARK/ETH 是算法组件候选，尚未复现其算法，不再将组件成绩当平台排名。完整 ASTM 与实体计量仍是物理可比成绩的前提。

用户最后澄清：baseline 先保留；benchmark 要使本平台性能与他人可比较。已补查 ASTM F2554-22（FDA 认可 11-415）及公开 SlicerAstmPhantomTest，见 [平台评价补查](research/PLATFORM_BENCHMARK_2026-10-23.md)。**下一步优先核对物理评价规范、模块和可匹配论文条件**；下方旧顺序中的 ETH／LARK 是组件研究，不应再被当成整机比较的主要答案。完整标准正文尚未取得，未安装模块、未做符合性测试。标准只覆盖静态点定位，头模目标、工具方向、动态误差需分别补充。

用户后续明确要求先研究已有平台、选择数据完整的 baseline 和 benchmark。本轮已完成 [平台与数据选型](research/BASELINE_REVIEW_2026-10-23.md)：严格完整平台本轮零合格；SlicerROS2 作软件参照，LARK 作头模光学主候选（外部数据许可待明），ETH ASL 作真实时空标定优先复现数据，PLUS 作组件回归。没有安装这些平台或复现论文数值。

接续顺序：读主报告及三份审计 → 固定小型数据与评价约定 → PLUS 组件回归／ETH 作者基线 → LARK 公开结果统计重算与视频复现依赖核查 → 选择有证据的误差问题。LARK 先区分结果表重算和原始算法复现；不要下载几十 GB 全包后才检查需要什么。正式纳入 LARK 数据集前核清许可；未授权向外部联系人发信。

[实物接入前研究_2026-10-23.md](实物接入前研究_2026-10-23.md)的完整坐标链、目标误差和时序有效性设计继续有效。头部影像＋模拟光学观测＋非单位变换＋独立仿真真值场景仍未实施，放在外部基线复现之后；不直接靠假设噪声宣布实际系统的主要误差来源。

既有机械臂场景没有头模，误差来自同一模拟模型，不是光学外部测量。算法比较与界面优化后移；退出限制作为工程维护。已有自治授权继续有效，无需重做安装或否定已通过的基础联调证据。

## 当前接续入口

- 直接验收：[使用与验收.md](使用与验收.md)；任务清单：[README.md](README.md)；完整证据：[RESULTS.md](RESULTS.md)。
- 最终闭环：`output/slicer_acceptance/run_20261022_165004/`，21 项检查通过，针尖位置 1.205062 mm、整体角 0.019561412 rad，4.062 秒保持内 20 次不同反馈，录像和 MRB 已保存。
- 简单障碍：`output/obstacle_validation/20261022_164147_c6cf4d/`，6 项通过，侧障执行、目标碰撞拒绝、移除后恢复。没有宣称绕障。
- 反馈中断：`output/feedback_disconnect/20261022_165114_5ce94f/`，最终面板代码 9 项通过，同一进程已恢复，测试盒体与锁均已清除。
- 启动／停止：`Start_UR12e_Demo.cmd` 等实际 ROS 与匹配 PID 的 Slicer 面板就绪；`Stop_UR12e_Demo.cmd` 等全部任务进程退出。最终冷启动已通过，旧目标未重放；日志自动归档到 `runtime/backend_logs/`。
- 当前仍有退出阶段异常：MoveIt 析构 -11，控制器曾需 launch 升级 SIGTERM；已核对进程组全部退出，不能称所有组件正常退出。见 `output/lifecycle_validation/final_20261022/` 和 `output/shutdown_review_20261022.md`。
- 后续先处理有实际需求的退出／跨系统通信边界；实物、真实工具标定和光学定位需要实际设备、网络、工具、机时及接口提供者信息，继续保留为任务第 4 阶段。不要重做已完成的安装，也不要把仿真精度当作实物精度。

## 2026-10-21 暂停记录（历史）

### 当时已完成

1. Windows WSL 3.0.3 与专用 `UR12e-Jazzy` 发行版部署完成；Ubuntu 24.04.5，数据目录 `D:\CodexEnvs\UR12e-Jazzy`。
2. ROS 2 Jazzy、MoveIt 2 2.12.4、UR 官方驱动 3.9.0、UR 描述 3.5.1 已安装并实际运行。
3. 真实 UR12e mock hardware＋普通 JTC＋规划服务／执行 action 已核对；弯肘 home 为 `[0,-1.3,1.6,-1.9,-1.57,0]`。针尖为 tool0 本地 +Z 120 mm，已加入简化碰撞体。
4. 机器人侧 7 项验收全部通过：规划不执行、缓存计划执行、消耗计划拒绝重放、横向运动、运动与转向、不可达目标不运动、失败后可达恢复。4 次实际执行均满足位置 ≤2 mm、整体角 ≤0.03 rad。
5. Windows Slicer 5.12.4 已接收真实 joint／TF 并显示官方机械臂与工具。Slicer 端完整目标设置→规划→执行验收和演示录像尚未开始。

### 当时证据入口

- 通过的机器人验收：`output/ros_runtime_validation/20261021_231842_b163ff/summary.json`，同目录保存过程观察、目标与执行事件。
- 实际 mock／控制器／ROS 图：`output/ros_environment_20261021/environment_summary.json`。
- Slicer 真实反馈截图：`output/slicer_ready_waiting.png`。
- 关机前快照：`output/paused_20261021/`，含最后 ROS 状态、Slicer 场景与截图。
- 之前 Windows 验证器未通过的记录：`output/ros_runtime_validation/20261021_231709_f52088/`，原因是跨 OS 壁钟偏差约 0.6 s，不能删去或写成整轮通过。后来在 WSL 同钟运行同一验证器整轮通过，未放宽 2 mm／0.03 rad／1 s 预算。
- 软件安装与下载日志：`runtime/ros2_setup/`、`runtime/prepare_linux_status.json`；421 个 ROS 包经官方索引 SHA256 核验后预取到 `downloads/apt_cache/`。

### 当时约定的继续顺序（现已执行）

1. 先看本文件、`README.md`、`RESULTS.md`、`validation/README.md` 和 `validation/SLICER_ACCEPTANCE.md`。不重做已完成的安装。
2. 修正机器人侧 `config/bridge.yaml` 与 `node.py` 中无效的显式 `RRTConnectkConfigDefault` 名称。当前本版官方 MoveIt 未声明此项，已通过的运行实际回退到默认 `geometric::RRTConnect`；可用空 `planner_id` 选择组默认，并在实际日志确认。暂停时尚未改此配置。
3. 用 `Start_UR12e_Demo.cmd` 启动；后端构建／安装缓存位于专用 Linux 的 `/root/ur12e_slicer_ws`。已修复 `setsid --wait`，避免 WSL 入口提前退出。启动后等待真实 `state.ready`、joint／TF 新鲜、针碰撞体和服务／action 均就绪。
4. 保持目标发送互斥：先核对后端，再将独占发送窗口交给 Slicer 完整验收脚本。脚本已准备在 `validation/slicer_acceptance.py`；通过面板从实测针尖设置附近静态目标（准备值为 Z -25 mm、yaw +3°），完成 plan→FK 预览→execute→4 秒稳定反馈核验。
5. 保存实际截图、场景、结果与 5 fps 演示帧，再用 `validation/encode_slicer_recording.py` 编码 MP4。Windows 媒体环境为 `runtime/media_env`，含已安装 ffmpeg；按帧的实际时间戳编码。
6. 补齐适合首轮的简单障碍物与断连反馈检查，更新主任务清单和结果；实物、光学定位、真实标定仍属后续阶段。
7. 本次暂停使用 SIGINT 停止后，节点均已退出，但 `move_group` 在退出阶段报告 -11，`file_bridge` 报告 -2；保留 `runtime/backend_stdout.log`。下次单独检查退出清理，不能把发出正常停止信号写成所有组件均正常退出。

### 持续有效的工程边界

- 这是官方 mock hardware 的通信／规划／仿真执行验证，不能据此宣称实物精度、真实工具标定或患者导航精度。
- Slicer RAS mm 与机器人 base m 的模拟关系当前为同轴同原点，实际坐标标定尚未接入。
- Windows／WSL 壁钟偏差需保留为联调限制；不从跨钟时间戳直接宣称端到端延迟。
- 后续所有实现和清单维护仍由 Codex 接管；用户恢复工作后继续推进。
