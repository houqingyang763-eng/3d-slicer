# 相近研究平台候选审计（2026-10-23）

状态：初步平台检索与公开材料审计，未安装第三方平台、未启动仿真/机械臂实验。GitHub 的当前默认分支、许可、文件树通过 connector 实时读取；只读了小型文本样例，没有下载 MRI、模型或原始数据包。源码可见不等于论文实验可复现。

## 判断与推荐

**本批未发现一个同时具备“头模 + 光学跟踪 + 工业臂 + Slicer + 完整代码 + 原始实验数据 + 独立精度评价”的完整合格 baseline。不能因为平台名称相同或数字较小而指定胜者。**

建议按可复用内容建立三层参照，最终主 baseline 还需结合 benchmark 审计综合决定：

1. **场景/工作流最近：OpenRoboTMS / SlicerRoboTMS。** 有 MRI 头模、光学标记、机械臂、Slicer 标定配准导航；但机器人算法后端当前不可公开访问，论文没有提供定量定位实验表与原始跟踪数据，暂不能作为精度数值 baseline。
2. **UR + 光学工程代码最近：RUGANI。** 有 ROS 2 Jazzy、UR5e、NDI、手眼标定、配准、数据记录；可读到真实小型校准/点坐标样例。但腹部超声任务、代码静态快照、没有可核验完整精度实验，不能替代头模端到端验证。
3. **独立实测与误差分解参照：Gao 等 UR10 + Polaris 脊柱注射。** 术后 CT 测针尖/方向，公开逐针表；但应用与头模不同，整套数据及控制栈未公开，不能直接复跑或横比针尖数字。

SlicerROS2/Micromate 的公开录制数据与教程由本轮另一路审计负责，以下只列候选，不重复声称其数据已经下载。

## 七个候选的比较

| 候选与原始入口 | 与项目相似处 | 精度/样本证据 | 公开性与主要缺口 | 本轮用途 |
|---|---|---|---|---|
| [OpenRoboTMS / SlicerRoboTMS，2026](https://arxiv.org/html/2604.25661v1) | FR3 + D455/AprilTags + MRI 打印头模 + Slicer；标定→配准→导航 | 原论文 IV 是示例集成；未见样本数、独立 TRE 表或原始日志 | Slicer MIT 代码/头模 MRI 已列文件树；ROS 后端链接 404 | 头模导航工作流参照 |
| [Fluoroscopy-Guided Robotic System for Transforaminal Lumbar Epidural Injections，2022](https://pmc.ncbi.nlm.nih.gov/articles/PMC10544812/) | UR10 + NDI Polaris + Slicer；光学闭环与多项标定 | 10 次机器人针 + 10 次医生针，术后 CT；机器人 5.1±2.4 mm、3.6±1.9° | 规划器公开；未发现整套原始 CT/光学/机器人日志公开入口 | 实物精度定义与分解方法 |
| [A semi-automated robotic system for percutaneous interventions，2023](https://link.springer.com/article/10.1007/s11548-023-02882-6) | ROS2/MoveIt2/Slicer + 八相机 ArUco + CT 腹部模体 | 4 路径；目标到实测路径均距 21.9 mm；光学位置误差均值 3.9 mm；闭环校正未稳定 | 论文完整可读，未发现作者平台代码/原始数据仓库 | 联调不等于精度成立的反例 |
| [SlicerFranka，2026](https://medcvr.utm.utoronto.ca/EMBC2026-SlicerFranka.html) | FR3 + SlicerROS2，平面/颅骨表面轨迹 | 作者页面报告 FRE 1.06 mm；没有外部光学跟踪 | 控制器/指针 CAD 可见；root license API 为 null，未见实验原始数据 | Slicer 操作与机器人接口参照，不作光学 TRE baseline |
| [ENTRI，2024](https://arxiv.org/html/2401.11715v1) | Slicer + AMBF + 光学配准的颅骨/鼻窦模体 + REMS | 1000 帧双程通信延迟均值 19.98 ms；不是端到端定位误差 | BSD-2-Clause 代码可见；ROS1，真实机器人/头模实验数据未在树中发现 | 双生场景/通信测量参照 |
| [RUGANI](https://github.com/PieceZhang/autonomous_needle_insertion) | Jazzy/MoveItPy + UR5e/FR3 + Polaris Vega VT；手眼/标记点配准 | 小型校准与点坐标文件可读；未见平台论文精度实验表 | Apache-2.0；作者标明 private 开发仓静态参考快照；大规模数据卡尚非公开数据证据 | UR 与光学集成代码参照 |
| [SlicerROS2 / ROS-MED Micromate，2024/2026](https://rosmed.github.io/embc2026/tutorial_part1) | SlicerROS2，真实光学记录、模体 CT、针 pivot、患者/机器人配准 | 公开教程重在导航场景搭建，不能把步骤演示当 TRE 数字 | 代码与工作坊数据需沿原始入口分别验证 | 软件骨架/坐标链回放候选 |

## 深审一：SlicerRoboTMS

已读：原论文 I–V（重点 III 的转换管理、IV 的硬件及集成）、作者项目页、README、INSTALLATION、Config.py、Calibration.py 的命令/状态段、完整递归文件树与 release API。

- 论文实物示例为 RealSense D455（1280×800、30 Hz）+ AprilTags + Franka Research 3 + MRI 生成打印头模；运行 Ubuntu20.04/ROS Noetic，不是当前项目的 Jazzy 原生组合。
- [GitHub](https://github.com/OpenRoboTMS/SlicerRoboTMS) 实时证实 public/MIT。当前 head 为 `e904691ff93e473eb535a28b21df93f162d21577`，有 `v1.0.0` release，release 附件为空。
- 文件树实际列出 `mri_data/mri.nii`（103,424,560 bytes），头/脑/注册笔/标记架 STL，FR3 URDF，Config 里五个头部 landmark 与默认目标。**MRI 和 STL 未下载或解码**；只能确认 Git 对象/大小，不声称影像质量与数据许可来源已核验。
- Calibration.py 的 `startCalibration()` 只向后端发 `start_calibration`；`calib_complete` 返回后显示相机。其测试覆盖状态/场景/连接，不验证手眼矩阵或 TRE。不能从 GUI 模块名称推断标定算法已完整开放。
- 作者主页 [ROSRoboTMS](https://github.com/OpenRoboTMS/ROSRoboTMS) 链接与对应 API 本轮均返回 404。可能私有/未释放/改名，不能判定原因。主页仍称代码整理中、部分资料待接收后释放；与前端已有 release 并不矛盾。
- 未在完整树中发现带时间戳的真实头部/工具轨迹、独立真值、评测器或 benchmark split。主页“毫米级/亚度级”只有示意曲线，无公开采样协议与样本数，不能采纳为可复核精度基准。

迁移 UR12e：可借其头模+工具+参考架的场景组织、标定/配准/导航分阶段、OpenIGTLink 消息约定。需要自己实现/核验后端转换、标定、控制，替换 URDF 与机器人驱动；不能把 ROS1 FR3 控制器直接搬来。

直接证据：[论文](https://arxiv.org/html/2604.25661v1)、[作者主页](https://openrobotms.github.io/)、[固定 head](https://github.com/OpenRoboTMS/SlicerRoboTMS/tree/e904691ff93e473eb535a28b21df93f162d21577)、[release](https://github.com/OpenRoboTMS/SlicerRoboTMS/releases/tag/v1.0.0)。

## 深审二：UR10 + NDI Polaris 脊柱注射平台

已读：原论文 II.B 系统标定、II 控制流程、III.A 校准、III 尸体实验与 Table II、IV 误差讨论；公开规划器 README 与完整文件树。

该论文最有用的是**测量层次分开**：机器人前向运动学不足时用 Polaris 跟踪注射工具相对底座参考架；光学坐标、器械和针各有校准；最终针尖/方向由术后 CT 与术前计划比较。30 个器械手眼姿态、6 幅针校准 X-ray、独立 10 针尸体结果，不能合为同一个“精度”。Table II 给出逐针值，可手动核对汇总；作者也指出换针没逐次重标定是误差来源。

[InjectionTrajectoryPlanner](https://github.com/htp2/InjectionTrajectoryPlanner) 为 BSD-2-Clause，固定 head `ff21a33395e873a5539298ca2829a5e64c0e54be`。树含 Python 规划器、针与器械 STL；README 明说样例影像/volume 因存储已移除，导入测试数据按钮无效。它公开的是规划前端，不能据此声称机器人/光学控制代码与全部数据开源。论文另一链接 [xreg](https://github.com/rg2/xreg) 是配准软件，仍不等于本实验数据集。

迁移 UR12e：可借“光学闭环校正＋独立术后/外部测量＋逐项校准验收”的研究设计；不必复制 C-arm/脊柱或尸体条件。头模应使用未参与配准的已知物理检查点/独立量测，不能把机器人 FK 当真实针尖真值。其穿刺误差受组织、针、X-ray 配准等因素影响，不能用 5.1 mm 直接判定本项目好坏。

## 深审三：RUGANI

已读：README、完整非截断递归文件树、两类真实小型样例（光学 CSV、CT fiducial TSV）、一份手眼 JSON、US-PPNR 数据卡。public/Apache-2.0；固定 head `14744dc8823c5d3cabfd001fca39cc96ed44cedd`。

可直接复核的材料：

- `registration/data/offline_polaris/abdominal_20250727_151056.csv`：9 行三维毫米点；`registration/data/preop/abdominal_CT.tsv`：18 个 RAS 中心点。数量不同，尚未核验哪九个点相互对应，不能直接计算配准成绩。
- `registration/slicer_module/RegisterOpticalMarkers.py`、`register_points.py`；`hand_eye_calib.py`；工具 follower/reporter、数据 recorder/rosbag decode/LeRobot 导出；UR5e 实物校准 yaml。文件存在已证实，算法与运行正确性本轮未测试。
- `calibration/hand_eye_20260112_071955.json` **target 是 `zed2`**；含 AX=XB 残差，中位平移 0.016664 m，P95 0.026717 m。它不是 Polaris 精度或独立导航 TRE；不能挑一个较小的旋转残差宣称高精度。
- 数据卡自称 US-PPNR 2,546 轨迹、10:51:09、6 操作者、CC BY4，但当前读到的卡没有公开下载 URL。频率叙述也有 30/60 Hz 不一致。只能作为待核验数据线索，不能声称获得公开 benchmark。

迁移 UR12e：机器人品牌与 ROS2 版本最接近，优先审计其 hand-eye 输入方向/单位、光学有效性/时间戳、工具偏移、标记点配准和记录格式。UR5e 的标定矩阵、具体工具/ROM、工作空间与关节设置都不能照搬；本项目光学设备型号尚未确定。

## 不宜忽略的反例与界限

Siegfarth2023 原文结果段说明：光学校正未稳定，因此四次实验只用相机测量，没有完成自动纠偏；同时可见 ArUco 数量不足，无法稳定测方向。它与我们技术栈很像，但“ROS2+MoveIt2+Slicer 联通”依然可能有厘米级偏差。这使其适合解释接实物风险，不适合充当高质量精度目标。

SlicerFranka 的 1.06 mm 是作者称 FRE、且无外部光学追踪；不得与独立检查点 TRE 混用。当前 repo API 未识别 root 许可，树也没有 root LICENSE；其中第三方 `franka_description/LICENSE` 不能自动授权自研模块，复用需再确认。

ENTRI 的约 20 ms 是 1000 帧通信 RTD，不能当“光学测量到机器人停止”的系统延迟，更不能把半个 RTD 当已实测单程；其原文只作约半延迟推断。ROS1 + AMBF 的迁移成本亦高于当前 UR+Jazzy 路线。

## 本轮留存与待补证据

机器可读证据见 [platform_sources_2026-10-23.json](platform_sources_2026-10-23.json)：GitHub 当前 SHA、许可证、完整树及读取的小型样例。无新增机械臂结果。

后续只有以下证据能改变本轮排序：OpenRoboTMS 完整后端/原始评测释放；发现具有公开追踪与独立真值的头模数据集；或 SlicerROS2/Micromate 数据审计确认可复用的真实标定/跟踪记录。暂不把未来自己生成的噪声仿真称作外部 benchmark，也不凭现成代码存在把本项目假设写成已验证结论。
