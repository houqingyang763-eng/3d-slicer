# 头模导航与评价程序：本轮成果

参考运行日期：2026-10-23；公开交付说明更新：2026-10-24。**已交付可运行的评价程序、MR 头模场景、模拟光学坐标链和真实 MoveIt/mock 执行。当前结果属于软件／仿真验证，不能与他人的实物精度直接排名。**

## 先看成果

- 已发布历史参考：[来源与范围](reference_results/2026-10-23/PROVENANCE.md)、[索引](reference_results/2026-10-23/index.json)。
- [离线误差报告](reference_results/2026-10-23/offline/report.html)、[MoveIt 联动报告](reference_results/2026-10-23/moveit_mock/report.html)、[完整联动数据](reference_results/2026-10-23/moveit_mock/run.json)。
- 新 Linux 机器按 [部署与验收说明](DEPLOYMENT_LINUX.md)安装，用 `bash ur12e_slicer/run_navigation_linux.sh` 生成本机报告与 Slicer 场景。
- 旧 Windows 查看 / 重跑入口为 [Open_Navigation_Benchmark.cmd](slicer/Open_Navigation_Benchmark.cmd) 与 [Run_Navigation_Benchmark.cmd](Run_Navigation_Benchmark.cmd)，需要原有 Windows + WSL 环境。
- 本机原始 MRB、截图、录像、临时会话与早期失败轮未随源码发布；Linux 入口会由公共 MR、模型和本机结果重建场景。下面的历史数值不代表接收方已在本机运行。

## 实际干了什么

1. **明确可比的测量对象。** [评价协议 v1](research/BENCHMARK_PROTOCOL_v1.md)审查了 ASTM 相关 Slicer 模块与三篇原始研究，区分静态定位参照、头模最终到靶、动态／有效性扩展；相同标准名称但条件不一致的结果未合榜。已有 baseline 保持。
2. **建立完整几何链。** 使用真实公共 `MR-head.nrrd`；影像与机器人间有明确的非单位旋转和平移。6 个合成配准点通过仅含旋转／平移的刚体拟合估计配准矩阵，5 个检查点不参与拟合。模拟相机分别观测头部参考架和工具标记，工具标记到针尖另有标定。坐标统一 mm，ROS 入口明确转 m。
3. **分离控制与评分。** 控制只用估计标定与模拟观测；真值预先保存并核哈希。另有 8 配准点／6 检查点的离线几何试验，用于逐项隔离误差，不把它当成影像中的解剖标注。
4. **实际跑 MoveIt。** 三次目标都经过 plan→缓存 execute→实际 mock joint/TF 回传，保留请求ID、action 状态、轨迹与非零关节变化。光学姿态由独立冻结的虚拟标定世界与 mock 反馈生成；它仍不是物理定位仪的独立测量。

## 最值得重视的结果

以下是预先指定扰动的确定性验证，不是实物误差大小或成因占比：

| 条件 | 可见现象 | 独立仿真真值 |
|---|---|---|
| 所有配准点共同偏移 2.5 mm | FRE 和估计显示误差可接近零 | 工具仍偏离目标 2.5 mm |
| 工具标定偏差 2.5 mm／3° | 共用错误标定的控制与显示可互相一致 | 最终位置／朝向仍有指定偏差 |
| 头模 20 mm/s，观测滞后 80 ms | 两路时间戳相同且尚未超过假设帧龄预算 | 当前目标误差 1.6 mm，等于速度×延迟 |
| 相机→底座平移偏差 3 mm | 机械臂到命令点可以很好，命令点却有偏差 | 评分保留标定偏差与实际控制残差向量，不能直接相加其长度 |

它们支持的工程结论是：**最终目标需要独立检查；FRE 小、显示到位、两路时间戳一致，分别都不足以证明最终定位正确。** 尚不能据此宣布本项目真实设备哪一项误差最大。

## 实际验收记录

| 检查 | 本轮结果 | 证据 |
|---|---|---|
| 数学与失效 oracle | 26 项全部通过，Slicer Python 3.12.10／NumPy 2.4.6 | [测试源](navigation_benchmark/test_geometry.py)，另保留 [独立实现首轮环境记录](reference_results/2026-10-23/checks/geometry_first_run_environment.json) |
| 指标与运动授权检查 | 10 项全部通过，包括整包旧时间重放、合法原点、无效数据为 null、bias 与 RMS 区别 | [checks](reference_results/2026-10-23/checks/metric_and_gate_checks.json) |
| 离线扰动／失效 | 12 类×6 检查点＝72 项；42 有效、30 按预期拒绝；15 项解析／哈希检查通过 | [run.json](reference_results/2026-10-23/offline/run.json) |
| 静态指标输入适配器 | 对称 ±1 mm 的输入：bias=0、mean/RMS/precision=1、span=2；9 个预期字段吻合 | [输入](research/benchmark_evidence/astm_audit/analytic_input_example.json)、[输出](reference_results/2026-10-23/offline/analytic_point_statistics.json) |
| 真实 MoveIt/mock 联动 | 3 次规划和执行全部通过；命令残差 1.325／0.912／1.790 mm；影像目标处误差 1.325／2.847／1.790 mm | [逐场景 CSV](reference_results/2026-10-23/moveit_mock/cases.csv) |
| 执行前失效 | 工具遮挡、旧帧、整包重放、时间不齐、规划后头模移动、标定变化，6 类均拒绝；没有写入执行命令，关节无变化 | 同轮 run.json 的 `rejection_cases` |
| 一键入口 | `Run_Navigation_Benchmark.ps1` 从离线评价到实际 Slicer 场景整轮完成 | [最新整包索引](reference_results/2026-10-23/index.json) |
| 场景重载 | 独立 Slicer 进程实际重新加载 MRB，保留非单位影像变换；6 个配准点／5 个检查点，目标在 MR 有效体素内（强度96） | [重载记录](reference_results/2026-10-23/checks/scene_reload_verification.json) |

原验收版本的源码、MRI、场景与图片哈希及检查索引见 [交付清单](reference_results/2026-10-23/checks/delivery_manifest.json)。原参考运行结束时已停止自己启动的 mock 后台和专用前端。清单中的本机场景 / 图片及源码哈希对应当时版本，不表示当前 Linux 移植代码或接收方已经运行。

三次规划采用旧阶段未放宽的 2 mm／0.03 rad 工程预算。它只是“到估计命令点”的 mock 验收，不是最终真实导航的误差要求；加入外参偏差时的 2.847 mm 不是失败后被删掉的结果，而是测试要显示的最终目标偏差。

首次未通过轮次保留在原作者本机 `output/navigation_benchmark/20261023_163353_3ded57/`（未发布）：两次执行后，Windows／WSL 共享文件原子替换期间出现瞬时 ENOENT。新适配器只对 ENOENT/EACCES 最多重读 0.25 秒，不放宽反馈新鲜度或精度阈值。后续通过轮次单独留存。

## 当前界限与下一步

- **标准比较：** 完整 ASTM 正文尚未取得，A 类规范实物采集未做。三篇论文仅能作方法／条件参照，当前没有可与本仿真直接合榜的实物成绩。
- **仿真：** 光学是模拟的，配准点／检查点是合成几何点；MR 等值面仅供显示，没有验证为头皮／颅骨分割。目标不是已标注解剖结构；没有向 MoveIt 添加头模碰撞几何。
- **运动授权：** 演示的是执行前应用层检查。直接向原通用文件桥发指令可绕过此 gate；未验证运动中失跟、硬件停止或停止延迟。
- **动态与保持：** 已知速度／延迟是离线解析场景，没有实物动态跟踪，也未为新头模链完成独立固定窗口的保持统计。
- **研究顺序：** 先落实实际相机／参考架／工具拓扑、完整标准与实体真值计量，再配置同条件物理测试。现阶段可继续增加目标位置／朝向覆盖，按同一任务保存重复试次；不能把本轮人为偏差直接当真实设备参数。

代码入口见 [几何与离线说明](navigation_benchmark/README.md)、[查看器说明](slicer/NAVIGATION_BENCHMARK_VIEWER.md)。基线选择的历史理由保留在 [首轮选型](research/BASELINE_REVIEW_2026-10-23.md)。
