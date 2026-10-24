# SlicerROS2 / SlicerIGT / PLUS 候选审计（2026-10-23）

审计范围：只读论文、官方文档、固定版本仓库，下载小型场景和标定数据并检查目录/头部；没有安装新平台、运行机器人或复现论文指标。

## 判断

**SlicerROS2 适合作为软件平台的主要对照候选，不能凭现有证据选为“光学导航定位误差”唯一 baseline。** 它与 Slicer + ROS 2 + MoveIt 2 最接近，公开代码和教程充分，维护到 Jazzy。其主要论文没有公开完整独立光学真值、误差分解和可重算的端到端误差数据。

可将证据分成三层：

| 对照对象 | 能作为哪种 baseline | 不能证明什么 |
|---|---|---|
| SlicerROS2 v1.2 | 图像、TF、机器人、MoveIt 集成的软件实现与复用参照 | UR12e + optical + head 的独立定位精度 |
| ROS-MED EMBC2026 教程 | 患者/针尖/相机/机器人坐标链的工作流模板 | 标准化评测集、隐藏真值、独立检查点或公平排行榜 |
| PlusLibData + PlusLib CTest | 具有输入和基线输出的标定/配准组件回归测试 | 临床精度、UR12e 精度或完整光学机器人系统误差 |

以上是该候选自身审计后的判断；尚不能据此声称它在所有平台中数据最全或质量最高。

## 2024 正式文章实际验证了什么

[正式 DOI](https://doi.org/10.1109/TMRB.2024.3464683) 与 [作者接受稿 arXiv:2509.19076](https://arxiv.org/html/2509.19076v1) 是同一项工作，不能计作 2024、2025 两篇独立验证。

论文侧重集成。通信测试每组 100 消息，往返 11.9±0.3 ms（50 Hz）、8.5±0.3 ms（100 Hz）；这是小消息软件往返，不是视觉到机械动作延迟。虚拟夹具用 Omni、伪肿瘤、4 名用户，场景含电磁定位。路径示例用 CT 舌体模、dVRK 和点配准，轨迹差 2.75±0.65 mm，测量来自机器人内部运动学；文中未交代该轨迹误差的独立重复次数、点数或独立 optical 真值。因此它既不是头模 + UR12e 实验，也不能直接对比我们的光学目标误差。环境为 Ubuntu20.04/Galactic/Slicer5.1.0，与当前 v1.2 有版本差。

## 代码、版本和本机迁移

实际核验 [v1.2 release](https://github.com/rosmed/slicer_ros2_module/releases/tag/v1.2) 为 2026-06-15：
- annotated tag 对象：`735207a8e5f8eb04654cb5e6a488cdbc79cb2cad`；
- 对应 commit：`8ae3e650276d7525eef3e49acd47a6aa05c395e2`；
- 递归 tree 共 187 条，未截断；未见 mrb/nrrd/nii/mha/csv/bag/db3/mcap 等实验输入；release 的 assets 为空；
- MIT 代码许可，必须保留许可和版权；论文版权与代码许可分开。

[官方兼容矩阵](https://slicer-ros2.readthedocs.io/en/v1.2/pages/compatibility.html) 指向 Ubuntu24.04 + ROS2 Jazzy + **源码编译的 Slicer5.10.0**。[安装文档](https://slicer-ros2.readthedocs.io/en/v1.2/pages/getting-started.html)要求 C++ 插件与 Slicer/ROS 同时链接，并使用系统 Qt/OpenSSL/bzip2；不能作为现成 Python 包直接塞入当前 Windows Slicer5.12.4。可另建 Linux Slicer 环境作对照；WSLg 可行性/图形与设备通路尚未本机验证，不应假定兼容即安装成功。

v1.2 已有 [UR mock + MoveIt + 自定义针尖例程](https://slicer-ros2.readthedocs.io/en/v1.2/pages/examples/moveit_custom_tool.html)，默认例程 UR 类型为 ur5，附针长 100mm；这是非常接近现有模拟功能的参照，但更换 UR12e、120mm 工具、当前驱动版本仍需独立核验。其自动换单位/约定不能代替实际手眼标定。

## 公开场景究竟有什么

[EMBC2026 Part1](https://rosmed.github.io/embc2026/tutorial_part1) 明确演示 spine phantom CT、针尖 pivot、患者到 optical tracker 配准；[Part2](https://rosmed.github.io/embc2026/tutorial_part2)再用两份 fiducial JSON 注册 Micromate。它比本项目第一轮模拟更接近完整导航链。但教程中的 `PatientRegistration.mrb`、原始 CT、robot/tracker 两份 JSON 位于预配置镜像的 `/home/user/workshop_data/`；目前没有查到这些特定文件的独立下载地址。不能仅因教程截图展示序列就写“光学原始数据已获得”。Dockerfile 含部分浮动 git 分支，也不是已经固定可复现的完整环境。

[Part3](https://rosmed.github.io/embc2026/tutorial_part3)链接作者仓库 [LauraConnolly/slicerros2_tutorial](https://github.com/LauraConnolly/slicerros2_tutorial)。本次核验固定 commit `29bcb2397475a8079c3a71dbd325545ae9978786`，tree25条，公开数据：
- `PhantomScene_wTransform_and_registrationPoints.mrb`：9,783,767B，已下载并读 zip/MRML；7个zip条目（含目录），含 spine mesh、5个配准点、静态变换；无 CT volume、tracking sequence、外部真值。
- `Spine_and_EE.mrb`：1,066,080B，已下载并读 zip/MRML；7个zip条目（含目录），含 spine mesh、UR5 stylus/goal、静态变换；无 CT volume、tracking sequence、外部真值。
- `RosBag_5x.db3`：67,141,632B；本次**只读 metadata.yaml，没有下载/解析 db3**。清单声明150,616消息、约1324.546s，topic仅 `/tf`、`/tf_static`、`/robot_description`、`/joint_states`。这不能证明它有 optical 观测或独立真值。

这些是**脊柱体模教材场景，不是头模**。作者数据仓库的 GitHub license 字段为 null，树中未见 LICENSE；不可把 SlicerROS2 的 MIT 自动套用到这些数据。可以先做本地阅读核验，后续纳入可分发 benchmark 需核清具体数据许可。

## PLUS / SlicerIGT 可复用程度

[SlicerIGT 官网教材](https://www.slicerigt.org/wp/user-tutorials/)将资料放在 OneDrive 共享目录。本次确认链接存在，但未成功枚举共享目录，不能声称已获得完整教材数据。

更明确的是 [PlusLibData](https://github.com/PlusToolkit/PlusLibData/tree/51dcbb76d9f29fad94cf80788a7a0b7c704fb5dc)，固定 commit `51dcbb76d9f29fad94cf80788a7a0b7c704fb5dc`，tree545条未截断，仓库 BSD-3-Clause。本次实际下载检查：
- `EightLandmarkPointsTrackedForPhantomRegistration.igs.mha`：730,336B；头部1000帧，只有 ReferenceToTracker、StylusToTracker 两类变换，状态均为OK；**仅凭头部未证实采集设备型号或实际/合成来源**。
- 对应 `PhantomRegistrationLandmarkDetection.results.xml`：含8个体模基准点及基线变换，适合组件结果回归。
- `PlusDeviceSet_fCal_Sim_PivotCalibration.xml` 明确是 **FakeTracker/PivotCalibration**；`StylusCalibration.results.xml`是其基线输出，不能当成真实光学测量精度。
- [PlusLib CTest 配置](https://github.com/PlusToolkit/PlusLib/blob/c97543721993d35b8afbe4c5edc5efa7c09d1080/src/PlusCalibration/Testing/CMakeLists.txt)明确把以上 seq/config 与 baseline-file 配对使用。当前只核验了资产和测试连接关系，尚未执行算法复现。

下一步建议先用有明确输入/输出的 PLUS 组件回归数据检验配准和标定实现，再用 SlicerROS2/EMBC 工作流建立项目级光学坐标链。机器人目标误差仍应在实际设备或具有独立真值的专用数据上验证；不能把教材场景包装为通用 accuracy benchmark。

## 本地证据

`benchmark_evidence/slicerros2_audit/` 保留：
- `source_manifest.json`：版本/树计数/数据清单/license；
- `mrb_manifest.json`：两场景逐项目录、MRML摘要、SHA256；
- `RosBag_5x_metadata.yaml`：只读包元数据；
- `plus_sequence_manifest.json`：帧数/变换名/状态/SHA256；
- 小场景原件、730KB序列、3份XML、PLUS数据许可。

本次未复现任何论文中的误差或时延数值，也未将这些教程数据升级认定为标准 benchmark。
