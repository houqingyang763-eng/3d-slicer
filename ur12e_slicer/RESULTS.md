# 阶段记录：环境部署与仿真联调

更新日期：2026-10-24。**基础联调与本轮 MR 头模／模拟光学链验证通过；没有真实光学或实物精度结果。** 当前仍有历史退出软件异常。新 Linux 机器按 [部署与验收](DEPLOYMENT_LINUX.md)；Windows 历史入口见 [使用与验收.md](使用与验收.md)。

## 公开结果与历史记录的区别

已发布的本周精选证据见 [参考结果来源](reference_results/2026-10-23/PROVENANCE.md)、[离线报告](reference_results/2026-10-23/offline/report.html)和 [MoveIt/mock 报告](reference_results/2026-10-23/moveit_mock/report.html)。它们来自原作者 Windows + WSL 环境，不替代新机器的实跑。下文较早阶段的 `output/`、`runtime/` 等路径用于定位原作者本机历史文件，除明确链接到 `reference_results/` 的副本外，不随源码发布；视频、MRB 和完整安装日志需本机重建或另外取得。

## 2026-10-23 评价程序与头模导航实际交付

完整结果见 [本轮成果](NAVIGATION_BENCHMARK_RESULTS_2026-10-23.md)。新增 [评价规范](research/BENCHMARK_PROTOCOL_v1.md)、几何／统计／授权代码、一键入口、MR 表面和三视图及自动报告。26 项几何 oracle、10 项指标／运动授权检查通过；12 类×6 检查点共72项离线评价，42有效／30预期拒绝，15项解析检查通过。三次真实 MoveIt/mock 执行通过，六类执行前失效均拒绝，未写执行命令且关节无变化。

最终一键参考运行已归档为 [离线记录](reference_results/2026-10-23/offline/run.json)与 [集成记录](reference_results/2026-10-23/moveit_mock/run.json)。命令残差1.325／0.912／1.790mm；独立仿真目标误差1.325／2.847／1.790mm，其中第二项故意注入3mm外参偏差。没有把它作为精度不合格后删去，也没有将控制残差误当最终到靶误差。

研究发现限于受控仿真：全部配准点同向偏移2.5mm可使FRE／显示误差接近零，而真值目标误差仍2.5mm；已知20mm/s与80ms滞后产生1.6mm当前误差。它们证明评价能发现指定故障，不给真实设备的误差来源排序。

没有执行 ASTM 的实体采集，没有标准符合性结论；无头模碰撞几何、运动中失跟／停止延迟或真实保持测试。首次共享文件 ENOENT 失败轮保留于 `output/navigation_benchmark/20261023_163353_3ded57/`；增加仅针对存储可见性的有界重读后另存通过轮次。

## 2026-10-23 相近平台与 benchmark 审计

完成 [平台与 benchmark 选型报告](research/BASELINE_REVIEW_2026-10-23.md)，附平台、数据、SlicerROS2/PLUS 三份审计。核对论文、固定版本源码、数据许可和实际可取得文件：包括 LARK 1.78 MB 结果表与原始大包目录／位置样例，ETH 小型真实姿态包，PLUS 1000 帧序列及基线 XML，两个公开 Slicer 教材 MRB。未全量下载大型视频／图像集。

本轮未找到同时具备头模＋光学＋机械臂＋Slicer＋完整代码／原始独立测量／评分器的单一完整 baseline。选择 SlicerROS2 为软件参照、LARK 为头模光学数据主候选（数据授权尚未明确）、ETH 为时空标定优先复现数据，PLUS 为组件回归。NousNav 和 UR10/Polaris 论文用于借鉴独立测量设计。

**这些是检索、文件内容和可复现性审计结果，不是算法或精度复现。** 论文中的均值／中位数未由本项目重现；没有新增头模／光学／机器人运行检查。下一阶段先外部基线复现，再基于可重复现象开展误差研究，详见报告的验收要求。

## 2026-10-23 范围澄清与研究调整

核对前端 `build_scene()` 和机器人 `feedback()`：机械臂本轮场景没有加载头部影像，RAS 到 base 为已知单位关系，针尖由模拟 joint／TF 和同一机器人模型计算，没有独立光学观测参与。验收中的“独立误差”是独立重算误差公式，不是独立外部测量；此前数值只能说明模型内的目标跟随与通信行为。既有头部影像配准练习和机械臂场景尚未整合，也未开展实体头模实验。

依据用户新要求，已完成代码审查、官方资料初查和三项优先研究方向筛选：[实物接入前研究](实物接入前研究_2026-10-23.md)。新增头模坐标链、光学模拟流、误差扰动与时序失效实验均未运行；本节记录研究进展，不增加已通过实验数量。下一阶段优先解决坐标／标定、目标误差和观测有效性问题。

## 2026-10-22 实际交付

| 检查 | 实际结果 | 证据 |
|---|---|---|
| 最终代码重启后的 Slicer 闭环 | 21 项全部通过；Slicer 控件设置目标→真实 MoveIt 规划与 FK 预览→引用缓存计划执行→实际关节／TF 回传→显示变换与独立误差核对 | results.json（本机历史：`output/slicer_acceptance/run_20261022_165004/results.json`，未发布） |
| 末端到位与静态保持 | 针尖位置 1.205062 mm，整体角 0.019561412 rad（1.120786°）；4.062 s 内 20 次不同反馈更新，整段满足 2 mm／0.03 rad，最大观测关节漂移 0 rad | 同上及 `observations.json` |
| 实际录像与场景 | 48 张真实窗口截图，约 13 秒／1600×960／平均约 3.67 fps；完整解码通过，未合成机器人运动 | MP4（本机历史：`output/slicer_acceptance/run_20261022_165004/UR12e_Slicer_MoveIt2_demo.mp4`，未发布）、截图（本机历史：`output/slicer_acceptance/run_20261022_165004/03_execution_complete.png`，未发布）、MRB（本机历史：`output/slicer_acceptance/run_20261022_165004/UR12e_closed_loop.mrb`，未发布） |
| 静态侧障与碰撞目标 | 6 项通过；侧障下真实执行误差 1.874127 mm／0.020828885 rad，7 个实际碰撞服务采样有效；目标盒与针发生碰撞时规划失败且关节变化为 0，移除后规划恢复 | summary.json（本机历史：`output/obstacle_validation/20261022_164147_c6cf4d/summary.json`，未发布）、障碍场景（本机历史：`output/obstacle_validation/20261022_164147_c6cf4d/UR12e_obstacle_scene.mrb`，未发布） |
| 反馈中断与恢复 | 最终界面代码的 9 项检查全部通过；真实暂停文件桥时控制禁用、显示淡化、位置与误差明确标为末次／无效，直接调用发送也被拒绝；同一进程恢复后重新就绪，其他 ROS 关节消息持续 | results.json（本机历史：`output/feedback_disconnect/20261022_165114_5ce94f/results.json`，未发布） |
| 实际软件图与规划器 | 官方 GenericSystem、普通 JTC active、服务／action 均核验通过；请求 planner_id 为空，实际日志为 geometric::RRTConnect，无无效配置名回退 | 环境（本机历史：`output/ros_environment_20261022/environment_summary.json`，未发布）、规划器（本机历史：`output/ros_environment_20261022/planner_summary.json`，未发布） |
| 启动与停止 | 启动入口等待真实后端就绪和匹配 PID 的 Slicer 心跳；停止入口等待任务组全部非僵尸成员及专属窗口退出。最终冷启动成功、旧目标未重放 | 运行检查目录（本机历史：`output/lifecycle_validation/final_20261022/`，未发布） |

场景解释与复现见 [障碍和中断验收](validation/OBSTACLE_AND_DISCONNECT.md)。侧方盒场景证明有障碍场景下的可达规划与碰撞目标拒绝；未声称演示绕障或连续碰撞证明。中断验收针对真实文件桥停更，不是实物断电、急停或网线拔除。

### 修复与保留记录

- 无效显式规划器名改为空，选择官方组默认 RRTConnect；已在实际参数和规划日志核对。
- 全窗口默认 PNG 压缩曾拖慢 Qt。录帧现使用实际抓图、缩至 1600 px、低压缩无损保存，保留抓取时间；4.594 s 的实测录制性能检查得到 21 帧、38 次不同反馈更新，最长单帧 0.183 s。视频编码另修复 Windows 相对路径及可变帧末尾时长问题，最终输出完整解码通过。
- `run_20261022_163634` 数值与动作成功，但保持阶段仅 4 次更新、15 帧，整轮仍为未通过；未删改。修复后 `run_20261022_163931` 已全部通过，最终代码重启后的 `run_20261022_165004` 再次全部通过。
- 障碍工具首次删除不存在对象被 MoveIt 拒绝、第二次遇共享文件瞬时 ENOENT，两轮未通过记录保留。独立验证器最多 0.25 s 重读 ENOENT/EACCES，仍执行原有 1 s 新鲜度预算；成功轮仅发生 1 次重读。
- 后端原子替换仅对短暂 PermissionError 做 80 ms 有界重试，不删除旧状态、不放宽新鲜度；3 项注入检查（本机历史：`output/atomic_write_retry_checks.json`，未发布）通过，已在最终代码重启运行。未声称消除了读侧跨文件系统的瞬时可见性问题。
- 重启复验曾发现“停止请求返回后旧进程仍在清理”，导致新启动被重复实例保护拒绝；过期反馈下 Slicer 正确拒绝发目标，记录在 `run_20261022_164725/start_rejected_stale_feedback.json`。现已加入退出等待、启动就绪检查、重复启动复用和日志归档，并实际冷启动通过。

### 退出阶段仍有异常

最新实际停止中，`file_bridge` 正常退出，任务进程组与 Slicer 均确认全部结束；但 `move_group` 在 `rclcpp::Executor` 析构时仍以 -11 退出，控制器曾由 launch 在 5 s 后升级 SIGTERM（-15）。因此“全部进程退出”不等于“所有组件正常退出”。停止入口不会自行发送额外强杀。

使用 `ros2 launch --noninteractive` 保证仅通知已校验的 launch 时，子节点仍会收到一次正常停止信号；随后最多等待 25 s，超时列出残留并失败。官方 [MoveIt #3680](https://github.com/moveit/moveit2/issues/3680) 有同为 Jazzy／2.12.4、同样析构栈的退出报告，这是相似旁证，未独立证明本机根因；没有为绕过它升级或修改大库。版本、来源和本机日志见 退出调查（本机历史：`output/shutdown_review_20261022.md`，未发布）。

当前运行中的仿真规划／执行／静态保持验收不受上述已观察的退出错误影响；退出修复仍列为后续工程事项。Windows／WSL 壁钟偏差也继续保留为限制，不从跨钟时间戳宣称端到端延迟或硬实时性能。

## 2026-10-21 部署与暂停结果（历史）

- 环境安装实际返回 0：Ubuntu 24.04.5／ROS 2 Jazzy／MoveIt 2 2.12.4／UR 驱动 3.9.0，包版本与日志保存于 `runtime/ros2_setup/setup-20261021T151114Z-419/`。
- 官方 mock／控制器／ROS 图核验通过：实际 URDF 使用 `mock_components/GenericSystem`，普通 JTC 激活，规划服务和执行 action 存在。证据：environment_summary.json（本机历史：`output/ros_environment_20261021/environment_summary.json`，未发布）。
- 机器人侧完整 7 项验收通过，4 次实际执行数据如下；验收线为针尖位置 ≤2 mm、整体四元数夹角 ≤0.03 rad。证据：通过的 summary.json（本机历史：`output/ros_runtime_validation/20261021_231842_b163ff/summary.json`，未发布）。

| 实际执行 | 独立针尖位置误差（mm） | 独立整体朝向误差（rad） |
|---|---:|---:|
| 缓存计划执行 | 1.7228 | 0.013151 |
| 横向运动 | 1.0413 | 0.020823 |
| 运动并转向 | 0.6522 | 0.015995 |
| 失败后可达恢复 | 1.7963 | 0.018912 |

- 规划不执行、消耗计划拒绝重放、不可达目标的最大观测关节变化均为 0；执行请求均收到 MoveIt 成功码 1 和 action 成功状态 4。
- 首次在 Windows 运行验证器因跨 OS 约 0.6 s 壁钟偏差而未完成，失败记录保留于 `output/ros_runtime_validation/20261021_231709_f52088/`；后续在 WSL 同钟运行原验收预算整轮通过。
- 实际规划器是默认 `geometric::RRTConnect`。当前显式 `RRTConnectkConfigDefault` 未被本版官方配置声明，因此发生回退；暂停时尚未修改，下次先修正配置并确认实际日志。
- Slicer 已接收真实关节／TF，显示官方机械臂和工具：真实反馈截图（本机历史：`output/slicer_ready_waiting.png`，未发布）。其完整目标设置→规划→执行验收和录像尚未启动。
- 关机前已保存 Slicer 场景（本机历史：`output/paused_20261021/UR12e_paused_checkpoint.mrb`，未发布）、截图（本机历史：`output/paused_20261021/slicer_paused_checkpoint.png`，未发布）与最后 ROS 状态；正常退出本任务的 Slicer，并向本任务 ROS 进程组发送正常停止信号。
- 停止后已确认仿真节点均不再运行。退出阶段 `move_group` 报告 -11、`file_bridge` 报告 -2；异常发生在已保存的运行验收之后，日志保留于 `runtime/backend_stdout.log`，下次检查退出清理。

上述误差是 mock hardware 内的目标跟随验证，受已设规划容差影响，不表示实物或患者导航精度。

## 已执行并观察到的结果

| 检查 | 实际结果 | 证据入口 |
|---|---|---|
| WSL 安装包 | 官方 WSL 3.0.3 x64，SHA256 与官方发布值一致 | `downloads/wsl.3.0.3.0.x64.msi`，`bootstrap/install_wsl.ps1` |
| 管理员安装 | MSI 返回 0；两个必需 Windows 特性均启用，并分别返回需要重启 | `runtime/wsl_install_status.json`、`runtime/wsl_msi_install.log` |
| Ubuntu 24.04 离线包 | 已下载，SHA256 与官方发行版清单一致 | `assets/ubuntu_source.json`、`downloads/ubuntu-24.04-amd64.wsl` |
| 重启与 Ubuntu 启动 | Windows 启动时间已晚于 WSL 特性安装；`UR12e-Jazzy` 导入成功，实际 `/etc/os-release` 为 Ubuntu 24.04.5/Noble | `runtime/prepare_linux_status.json`、`runtime/prepare_linux_*.log` |
| ROS 下载加速 | Linux 直连 GitHub 超时，改用 Windows 预取官方源配置包；另预取官方 apt 索引中 421 个 ROS 包，逐包大小/SHA256 通过并导入 apt 缓存 | `assets/ros_apt_source.json`、`runtime/apt_package_uris.json`、`runtime/apt_prefetch_20261021_230740_19744.json`、`runtime/apt_cache_import.json` |
| UR 模型资源 | 官方 Jazzy 描述仓库已下载，固定 commit `c2ae8652b6e9c1f9f3fedac48d63bef93d3cb9ba` | `assets/description_source.json`、`assets/ur_description/` |
| Slicer 前端 | 实际 Slicer 5.12.4 的 13 项前端检查通过；官方模型资源加载、单位与朝向、无反馈禁发命令、旧计划限制、保存/录帧、过期提示均已检查 | `output/frontend_verification/results.json`、`output/frontend_waiting_for_ros.png` |
| 协议与几何组件 | 7 项非 ROS 单测通过；Python 编译与 package.xml 解析通过 | `output/protocol_checks.json`、机器人包 `tests/test_protocol.py` |
| Windows 启动脚本 | PowerShell 语法解析通过；整体启动入口在未重启时明确退出 | `Start_UR12e_Demo.ps1` |
| 续装入口的重启门控 | 重启前验证返回 3010；重启后已通过门控并进入 Ubuntu/ROS 安装 | `output/pre_restart_summary.json`、`runtime/prepare_linux_status.json` |
| Bash 脚本 | Git Bash `bash -n` 静态语法检查通过 | `bootstrap/setup_ros2.sh`、`run_simulation.sh`、`stop_simulation.sh` |

安装包、日志、下载的第三方资源与运行目录均保存在本机，按任务目录 `.gitignore` 排除；来源清单、代码与本说明保留在任务目录。

本阶段汇总快照：output/pre_restart_summary.json（本机历史：`output/pre_restart_summary.json`，未发布）。等待 ROS 的场景已保存为 `output/pre_restart_waiting.mrb`；本任务启动的 Slicer 已正常退出，未强制终止其他窗口。

## 重启前的代码准备记录（历史阶段）

- 官方 UR12e mock hardware＋MoveIt 2 的启动配置；默认 RRTConnect，普通 `joint_trajectory_controller`。
- Slicer 与 ROS 的双向 JSON 文件协议；目标、关节与 TF 反馈、针尖位置和朝向误差。
- 简化工具沿 tool0 本地 +Z 延伸 120 mm，目标表示针尖位姿；ROS 侧加入同尺寸简化碰撞体。
- 规划预览、规划后执行、规划并执行、真实规划轨迹的 FK 采样，以及启动旧目标抑制和超时处理。
- 统一启动、专用发行版安装、场景保存与演示帧记录入口。

重启前没有 ROS 规划／执行结果；该历史阶段的 `frontend_waiting_for_ros.png` 只证明前端启动，`frontend_verification/frontend_fixture_only.png` 使用明确标注的隔离测试数据，仅验证显示和界面行为。重启后的真实结果以上方“本轮最新结果”为准。

## 首轮后的工作

1. 【首轮完成】环境、机器人侧验收、Slicer 完整闭环、实际录像、简单障碍与中断反馈。
2. 【后续工程】继续解决已记录的退出异常和跨文件系统／跨时钟边界；按需要增加场景变化、轨迹约束和算法比较。
3. 【实物阶段】取得实际控制器、网络、工具及可用机时信息，先接状态回传，再逐步进行受控运动和实测。
4. 【标定／光学定位】确定接口提供者与设备协议，将模拟变换替换为有来源的实际标定，独立验证工具和影像／跟踪坐标误差。

以上继续工作由 Codex 承担，按用户确认的首轮范围完成实际联调、结果记录与演示交付。
