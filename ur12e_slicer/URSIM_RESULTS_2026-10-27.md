# URSim 基础控制与 Slicer 闭环：2026-10-27

本文是 10 月 27 日历史通过记录。10 月 31 日两轮复跑 Slicer 20/21（录帧 19<20），外层未全通过；最新情况见 [项目上下文](../PROJECT_CONTEXT.md)。三轮 [精选 JSON](reference_results/2026-10-31-ursim/README.md)已归档，本文的本机 `output/` 链接不表示原始大量文件全部上传。

本次已完成 **UR12e URSim → ROS 2 / MoveIt → Slicer → URSim 反馈** 的本机软件验收。正式完整轮为 **`20261027T044855Z_f8a2ba`**，执行 `bash ur12e_slicer/run_ursim_linux.sh --stop-after` 退出 0，基础控制 5 项、Slicer 21 项和验收停止无残留检查同轮通过。

这是 Windows 宿主 WSL2 / WSLg 中的 Ubuntu 24.04、普通用户、Linux Slicer 5.12.4 与本机 Docker 虚拟控制柜运行。复用了既有 ROS / MoveIt 环境和模型资源；没有连接真机或物理光学定位仪。下述误差均由虚拟模型的关节 / TF 反馈计算，没有实物精度、临床或 ASTM 合规结论。

## 1. 本轮环境与执行范围

| 项目 | 本轮记录 |
|---|---|
| URSim | 官方 PolyScope 5.23.0 Linux amd64，固定镜像 digest `sha256:f1211a7feb4252135b1439f603f4e17ff826b4b71574f3963c868efe0498b4a9` |
| 机器人 | ROS `ur12e`；容器 `ROBOT_MODEL=UR12`，使用官方型号映射 |
| External Control | 1.0.5；主机 `192.168.62.1:50002`，保存 UR12 默认安装并启用 Remote Control |
| ROS | Jazzy / MoveIt 2；`scaled_joint_trajectory_controller`；domain 62、localhost only、系统时间 |
| 桥来源 | `ros2_ursim`；原生 Linux cache 下的 `ursim/bridge/` |
| 工具 | `tool0` 局部 +Z 120 mm 简化针，未作真实工具标定 |
| 完整运行 | `no_slicer=false`、`validation_exit_code=0`、`slicer_passed=true`、`stop_passed=true`、`passed=true` |

来源和首次部署见 [URSim 部署说明](URSIM_LINUX.md)与 [资源清单](assets/ursim_source.json)。照片提供的 Baseline 14.0.514、PolyScope 74.27.218、Controller 99.4.47、Firmware 61.0.133、Platform 0.1.1 与所选 URSim 软件包的准确对应仍未确认；本次选择提供 UR12e 支持，不宣称精确复刻真机软件。

相关离线回归为桥 / 协议 18 项、URSim 生命周期 / 归档 26 项、URSim 验证器 24 项、WSL guardian 6 项，共 **74 项通过**；原有几何 26 项、资源 4 项、旧生命周期 12 项及指标检查器 10 项也通过。这些是软件检查，实际控制仍由下述同轮结果证明。本机日志位于 `output/ursim_setup_2026-10-27/offline_regression.log`、`offline_existing_checks.log` 和 `archive_regression.log`；guardian 检查单独执行。安装入口随后幂等重跑退出 0，缓存 / SHA256 / 镜像核验通过，日志为 `output/ursim_install/20261027T045127Z.log`；没有将安装复跑当作另一台新机器首装。

## 2. 基础控制 5 项

原始 action / joint 记录与 [基础汇总](output/ursim_validation/20261027T044855Z_f8a2ba/20261027_124939_313242/summary.json)属于本机输出，不随源码交付。三次关节动作均由实际 `/scaled_joint_trajectory_controller/follow_joint_trajectory` 接受，action status 4、controller error code 0，并读取边界之后的新鲜 `/joint_states`。

| 检查 | 本轮结果 |
|---|---|
| 弯肘初位 | 通过；最大关节绝对残差 `1.215712e-5 rad` |
| 第一关节 +0.05 rad | 通过；最大关节绝对残差 `1.386403e-5 rad`，实际观测到位移 |
| 返回初位 | 通过；最大关节绝对残差 `3.984647e-6 rad`，实际观测到返回 |
| 针尖 +Z 10 mm，仅规划 | 通过；MoveIt 返回 4 个轨迹点，规划阶段最大关节变化约 `5.15e-13 rad` |
| 执行缓存计划 | 通过；针尖位置残差 **1.615011 mm**，整体角残差 **0.018512 rad** |

基础轮接收 10579 条关节消息、419 条 action feedback。文件桥末态与额外读取的新鲜实际关节反馈最大差约 `6.95e-12 rad`。上述重算只用于检查软件数据一致性，仍使用同一虚拟机器人与运动学模型，不能称为外部独立物理测量。

## 3. Linux Slicer 21 项闭环

本轮 [Slicer 结果](output/ursim_slicer_acceptance/run_20261027_125004_106100/results.json)为 `source=ros2_ursim`、`all_passed=true`，21 项 checks 全部为 true。它通过本轮 Slicer 控件生成目标、实际 MoveIt 规划预览、执行已缓存计划、显示虚拟控制柜反馈，并检查规划不运动、请求 / 计划对应与保持反馈。

| 指标 | 本轮结果 |
|---|---|
| 最终针尖位置残差 | **1.705336 mm** |
| 最终整体朝向角残差 | **0.013216 rad**（约 0.757229°） |
| 本次工程判据 | 位置 ≤2 mm；整体角 ≤0.03 rad |
| 保持窗口 | **4.170023 s**；22 个观测样本、21 次不同反馈更新 |
| 保持窗口最大位置残差 | 1.705336 mm |
| 保持窗口最大整体角残差 | 0.013216 rad |
| 保持窗口最大关节漂移 | 约 `7.35e-13 rad` |
| 录帧 | 20 帧；录帧在保持窗口之前结束，保持过程仍独立采样 |

本机保存了 [执行完成截图](output/ursim_slicer_acceptance/run_20261027_125004_106100/03_execution_complete.png)和 MRB。截图、录帧与场景服务于软件验收，不能替代 ROS action、实际关节与误差记录；没有宣称用虚拟残差预测真机精度。

## 4. 本周调试过程与方案

| 问题 / 观察 | 本次处理与证据边界 |
|---|---|
| DrvFS 高频读取 `ENODATA` 瞬态 | 保留失败轮；仅对存储瞬态作有界重试，随后将 URSim 高频 `state` / `target` / 验收锁 / 事件移至原生 Linux cache 的 `ursim/bridge/`，基础原始日志移至 `ursim/validation/`；没有放宽有效性 / 新鲜度判据 |
| WSL wall clock 周期回退 | 本机观察约每 31 秒回退 0.832 秒；专用 `UR12e-Jazzy` 发行版停止并禁用 `systemd-timesyncd`，保留宿主 Hyper-V 同步；42.017 秒采样无回退。日志见 `output/ursim_setup_2026-10-27/wsl_clock_adjustment.log`，恢复命令见部署说明；不推广为原生 Ubuntu 通用步骤 |
| ext4 到 DrvFS 归档元数据 `EPERM` | 保留基础动作已通过、外层归档失败的轮次；归档由带元数据复制改为 `copyfile` 字节复制，逐文件对比 SHA256，保留 `archive_provenance.json` 和原始位置 |
| Docker 29 内部桥 GUI 端口发布不可用 | 保留内部隔离网络，增加只绑定 `127.0.0.1:6080` 的本机 relay，转发到已核验的容器；未开放实验室网络 |
| 空闲 WSL 会话退出后 Docker 不再持续 | 提供 Windows `Start_URSim_Demo.cmd` / `Stop_URSim_Demo.cmd`、本机 WSL 配置和会话保持脚本；启动派发与真实就绪分别核对 |

高频原始记录先在 ext4 运行，成功或失败后都逐字节归档。正式轮 [外层汇总](output/ursim_validation/20261027T044855Z_f8a2ba/lifecycle_summary.json)和 [归档来源](output/ursim_validation/20261027T044855Z_f8a2ba/archive_provenance.json)分别保存公开可读结果位置与原始本机位置。操作日志位于 `output/ursim_setup_2026-10-27/` 和 `runtime/ursim_logs/`；这些目录含本机信息，不纳入源码发布白名单。

## 5. 停止检查与退出限制

正式完整轮 `--stop-after` 返回成功，停止检查时本任务 backend、Slicer、GUI relay 和容器均不运行。该记录证明这轮已结束并清理本任务组件；之后可重新使用 Windows 一键入口启动演示，当前是否运行以 status 的最新结果为准。

Windows 包装器另作了实际双启动与停止检查：第一次启动后 ROS、Slicer 和 GUI 均就绪；第二次返回 `guardian_reused=true`，没有留下第二个守护进程。执行 Windows 停止入口后，记录的 guardian 已退出，本任务四类组件均不运行，两个 Windows 启动进程也均已退出。本机证据位于 `output/ursim_windows/20261027_125427_468.stdout.log`、`20261027_125428_031.stdout.log` 和 `output/ursim_setup_2026-10-27/windows_guardian_{ready,stopped}.json`。临时调试用的会话保持进程也已清理。

本次过程 / 停止日志仍记录 Docker `137`、MoveIt `-11`、ros2_control `-15` 等异常退出信息。它们保留为已知工程限制，不能写成全部组件无错误退出，也不能由无残留推出硬件安全停止、停止时延或真机控制性能通过。

## 6. 后续安排

本周机器人组的 URSim 环境、基础控制程序和 Slicer 联动已有可复跑结果。继续演示使用 Windows 一键入口或 Linux 独立启停入口；汇报保留版本、实际控制步骤、量化结果和上述问题 / 处理过程。

后续真机实验仍需与博士生协调统一机时，并确认真实控制柜软件、本体校准、工具 TCP / 负载、网络和现场操作条件；光学导航另需定位仪、参考架、影像 / 工具标定和实体独立计量。本轮没有复跑旧 mock 的整套 72 个头模离线场景、三执行 / 六拒绝作为 URSim 头模评价，也没有完成导航方向点面配准或动态目标跟随的新实验。
