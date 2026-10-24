# 可复用 benchmark 核查（2026-10-23）

用户后续澄清要求跨平台性能可比性，本文主要是算法组件数据审计，范围不足以回答整机比较。**最新平台评价方向见 [ASTM F2554-22 与跨平台 benchmark 补查](PLATFORM_BENCHMARK_2026-10-23.md)**。下文“完整公开 benchmark 未发现”不表示没有标准化物理测试方法。

本文件只做候选筛选、原始发布物核验与使用边界分析。没有运行标定算法、训练模型或接机械臂；没有下载 23.5 GB / 25.9 GB / 50.1 GB 的完整数据集。完整平台文献由主报告另行汇总。

## 结论

- **完整平台公开 benchmark：本轮检索未发现合格项。** 合格应同时覆盖机器人关节/末端、独立光学跟踪、头模影像与标记、标定关系、时间戳、独立目标测量和明确评价程序；下面各项都只覆盖子问题。
- **光学定位与头模误差：LARK 最贴近当前问题，建议先复核其已发布结果与评价程序。** 数据、代码、网格和头模均有公开入口，且把配准点与检查目标分开；但无机械臂，原始位置流无显式时间戳，不能直接检验延迟。外部数据 ZIP 没有附许可证，代码/设计 CC BY 4.0 不宜自动扩展为数据许可已完全明确。
- **机器人与外部测量间的时空标定：ETH ASL 2017 是优先实测子基准。** 文件很小，既有真实 UR10/相机，也有 PrimeSense/Vicon 两条带时间姿态流。真实记录没有独立已知精确外参，检验应使用留出运动一致性与时偏误差，不能把优化残差冒充绝对标定精度。
- **可控误差与退化：KinHEC 可作补充合成基准。** 有精确 X/Y/时偏及退化、掉帧、外点、机器人模型误差，但 2026-09 才发布，尚不能视为成熟独立验证标准。METRIC 作为图像端/相机网标定的第二候选。

## 候选总表

| 候选 | 覆盖任务 | 数据与真值/参考 | 当前访问与许可 | 建议 |
|---|---|---|---|---|
| [LARK 2026](https://nist.mni.mcgill.ca/data/) | 光学工具定位、头模配准、遮挡/相机数 | 正式 20 网格试验＋20 头模试验；独立检查点；几何参考；同时采 fusionTrack 500 | 结果 ZIP 1.78 MB 已核；原始视频 ZIP 23.50 GB 已核目录；[代码/设计 CC BY 4.0](https://codeberg.org/IbisNeuronav/LARK/src/branch/main/LICENSE)，数据单独授权待确认 | 最贴近的 tracking/registration 子基准 |
| [ETH ASL hand-eye 2017](https://projects.asl.ethz.ch/datasets/hand-eye-calibration-2017/) | 空间外参＋时偏估计 | 真实 UR10/相机；Vicon/PrimeSense；另有仿真 | [GitHub 小 CSV ZIP](https://github.com/ethz-asl/hand_eye_calibration/tree/master/datasets)已下载核对；[DOI](https://doi.org/10.3929/ethz-c-000788527)元数据为 In Copyright - Non-Commercial Use Permitted；代码 BSD 3-Clause | 非商业研究优先；真实外参绝对误差无精确真值 |
| [METRIC 2023](https://zenodo.org/records/7976757) | robot-world hand-eye＋相机网 | Panda＋4 相机；真/合成图像；real 的参考外参来自 AprilTag＋mount CAD＋FK | 25.94 GB ZIP；CC BY 4.0（Zenodo API）；目录和小 CSV 已核 | 第二阶段测试图像前端与距离变化；real 参考不独立于 FK/视觉 |
| [Koide–Menegatti 2019](https://researchdata.cab.unipd.it/122/) | eye-in-hand 图像标定 | calib_test 与 spirit_reconst；机器人位姿、图像、手工特征；32 mm 图案 | 15 MB tar.gz；CC BY 4.0；landing 链接有效，未下载 | 用重建平面度/重投影验证；不是光学跟踪器/头模/机器人终点真值 |
| [RIRE v3](https://rire.insight-journal.org/) | CT–MR / PET–MR 影像刚性配准 | 植入标记建立隐藏参考变换，训练/测试分开 | 官方称图像开放下载；明确旧在线评价服务已不维护、真值仍隐藏；本轮未核数据许可与IPFS逐文件可用性 | 当前不作为可独立重现的头模导航 benchmark |
| [KinHEC 2026](https://huggingface.co/datasets/ezharjan/KinHEC) | 合成 AX=XB / AX=YB、时偏、退化、外点 | 77,628 序列，全精确变换/噪声标签；29,850 eye-to-hand | HF公开非 gated，MIT；card/API/license已核；完整50.1 GB未下 | 补充压力测试；不能证明真实系统精度 |

## LARK：最贴近头模与光学误差，但不能夸大

来源：[作者原文 §4、Appendix B](https://arxiv.org/html/2610.07561v1)、[实验室 data](https://nist.mni.mcgill.ca/data/)、[代码](https://codeberg.org/IbisNeuronav/LARK)。论文是 2026-10-06 预印本；本轮未查到其接受/发表版本。

核验到的正式实验是：网格每次 9 个配准点、16 个独立目标、4 条槽轨迹；头模每次 9 个解剖配准点、10 个独立颅部目标、2 条槽轨迹，各 20 次。指标为 mm 的 FRE、TRE、最近邻轨迹 RMSE；后者只量空间接近程度，不量时间延迟或沿路径落后。网格遍历 31 个相机子集，头模 top/left/right 遮挡是从同一五相机录像里去掉视角。

**实际文件审核：**

- `LARK_Datasets.zip` SHA256 `6f41e55fb459b1677e8b3514f628ad1504b3764eb6b68df93e9613b406fd7214`。含 Grid 99,760 行和 Head 34,440 行结果，外加 data dictionary。结果表不是原始姿态数据。
- 对原始 `Videos_release.zip` 使用 HTTP Range 只读取 ZIP64 中央目录与两个小 txt，完整包未下载。目录含 5,225 个 MP4、1,045 个 IGTPos；多出的 `GT21` 有 45 组。正式统计应排除这个额外试验，采用 GT01–20、HT01–20，即 1,000 次采集 × 5 相机＝5,000 个视频，不能称 5,000 个独立重复试验。
- 两个实取 `IGTPos.txt` 样本都为 150×3 的针尖 xyz，单位 mm；无显式时间戳/旋转列。它在 fusionTrack 的静态参考架坐标中，与 LARK 参考架坐标不同。跨系统需要配准，不能直接相减。
- 代码提供 9×3 `registration_points.txt`、10×3 `target_points.txt`、轨迹参考和头模 STL 入口。网格参考来自加工几何，头模参考来自 MRI 派生模型；fusionTrack 是独立比较测量，不是零误差真值。打印、CAD与实物差异、工具尖端标定、配准和商业跟踪器仍影响最终误差。
- 仓库 commit `5e80e3ee9c90f7b6b9bb7e2a08c2017359854286`；仓库 LICENSE 和 README 明确 CC BY 4.0，README措辞特指 code。两个外部 ZIP 内均未发现 LICENSE；在改编/再分发数据前澄清其授权覆盖范围。

**划分与使用：**没有机器学习 train/test split；配准 fiducial 与独立 target 已由协议固定。若以后调滤波/拒绝阈值，应按整次 Trial 留出，不能把同一次采集的视角子集、连续帧或重复的五相机无损条件分到训练和测试两边。先复核发布结果和统计定义，再考虑在真实原始数据上比较方法。仅用当前小 CSV 不能重算一套新的跟踪算法。

### 评测程序确实公开，但本项目尚未运行复现

并非只有 GUI：[data_collection.py](https://codeberg.org/IbisNeuronav/LARK/src/branch/main/Tracking/data_collection.py)处理录像生成跟踪 JSON；[data_processing.py](https://codeberg.org/IbisNeuronav/LARK/src/branch/main/Tracking/data_processing.py)计算配准和误差；[summarize_head_points.py](https://codeberg.org/IbisNeuronav/LARK/src/branch/main/Tracking/summarize_head_points.py)从 CSV 重建头模点表；[paired_statistics.py](https://codeberg.org/IbisNeuronav/LARK/src/branch/main/Tracking/paired_statistics.py)以 trial 配对做检验、去掉重复五相机条件并输出输入哈希；[recalculate_atracsys.py](https://codeberg.org/IbisNeuronav/LARK/src/branch/main/Tracking/recalculate_atracsys.py)用原始 IGTPos＋几何参考重算商业跟踪器。仓库还提供相关 tests。**本轮只审阅入口与代码，不等于执行通过或重现论文数值。**

### 数值必须保留统计口径

原文 §5、Table 1 和 Fig.8–11 明确采用不同观测单位。Table 1 先在每 trial/config 内把点误差或轨迹样本汇总成 RMSE，再报告这些 RMSE 的中位数与标准差；图中的点分布直接使用单目标误差，轨迹分布使用每条曲线 RMSE。因此摘要/图中的中位数与 Table 1 不能互换，差别本身不等于论文错误。

| 五相机＋adaptive，单位 mm | 摘要/图的中位数 | Table 1 的 median ± SD |
|---|---:|---:|
| Grid，triangulation，点定位 | 0.64 | 0.7477 ± 0.0513 |
| Grid，pose fusion，轨迹 | 0.73 | 0.7675 ± 0.1035 |
| Head，triangulation，点定位 | 1.13 | 1.2774 ± 0.1938 |
| Head，triangulation，轨迹 | 1.06 | 1.1197 ± 0.1641 |

这些都是作者发布值，本项目尚未复现。正式比较应固定一套主统计，并分别重建原文表和图；推断统计的独立重复单位是20次采集 trial，不是13万行误差或5000段视频。

## ETH ASL：真实两条姿态流，可先研究时空标定

[官方代码 README](https://github.com/ethz-asl/hand_eye_calibration)定义行格式为 `t[s], x[m], y[m], z[m], qx, qy, qz, qw`（Hamilton quaternion），本轮实际文件没有标题行。GitHub commit `966cd92518f24aa7dfdacc8ba9c5fa4a441270cd`。

| 已核小包 | 原始行数 | 时间范围/说明 |
|---|---|---|
| robot_arm_w_color_camera_real.zip（265,177 B） | robot 2,817；视觉 target 1,703 | robot约56.32s，UR10＋RealSense SR300；两个流起止时间不同 |
| prime_sense_color_w_vicon_1.zip（468,335 B） | Vicon 7,093；视觉 1,533 | Vicon约70.93s；含独立外部光学测量流 |
| robot_arm_w_color_camera_sim.zip（114,495 B） | hand 3,488；camera 1,067 | 仿真可以有模型精确外参；原始 hand 开头有重复时间行，读取要先检查 |

真实 robot 包 SHA256 `2454578f731e656a940ddf51017b56e8d535c58d16a50629b85326005dedd6c3`。这些包没有头模、影像配准、患者参考架或真实工具终点。代码的 BSD 3-Clause 不代表整个 DOI 数据权利相同；[DataCite 元数据](https://api.datacite.org/dois/10.3929/ethz-c-000788527)明确为非商业使用允许的版权状态。

没有官方 train/test split；未来应先划分绝对姿态/连续时间段，再构造相对运动，防止同一个姿态同时进入拟合和评估。真实数据先报告留出运动的一致性、可用时间重叠、估计时偏及退化程度；有精确 GT 的仿真才报告绝对外参误差。转换到我们系统时，要核清外部相机静置、marker随法兰运动的 eye-to-hand 定义，不能直接照搬 eye-in-hand矩阵名称。

## METRIC：有真实参考外参，不能说只有合成 GT

[论文](https://doi.org/10.3390/info14060314) §3.2 明确真实参考变换由 15 cm AprilTag、定制支架 CAD、机器人 FK、AprilTag 检测共同计算；真实包也确有 `GT/gt_cam*.csv`。因此可比参考外参，但不构成独立计量精度保证。合成部分的外参来自 Gazebo 精确设置。

Zenodo API 核对：CC BY 4.0；包大小 25,944,578,404 B；原发布 MD5 `a319a7549bc2246bcb97f53b59d7526b`（完整包未下，未本地重算 MD5）。HTTP Range 核到 36,555 个目录条目、18,072 PNG、18,144 CSV、72 YAML；抽取真实 robot pose 和参考 GT 为有效 4×4 矩阵。目录有 real/synthetic、sensor、pattern、cell、camera/image/pose/intrinsics/GT 层次。

指标为平移差 mm、旋转差 deg；没有独立机器人＋光学tracker姿态流、没有头模/工具TRE，也不是时序数据基准。距离/相机类型是其价值，若研究现有商用光学跟踪器外参，优先级低于 ETH 实测姿态流。

## KinHEC：误差注入的补充，需自定严格留出

[数据 card](https://huggingface.co/datasets/ezharjan/KinHEC)与API、LICENSE均已核。HF commit `9cc2e2642c0ef86233c9e1edff889d6af14bcdaa`，由 kinhec 1.0.1、seed 20260909 生成。含 8 tracks，特别有 temporal、motion degeneration、robot_error、outliers；GT包括X/Y/时偏、真实关节/观测、可见性与外点。

单位 m、s、rad，存储四元数是 `qw,qx,qy,qz`，与 ETH 不同。评价为外参平移 mm、旋转 deg、时偏误差等。HF只有 `train` split，不能把其名字解释为已提供独立训练/测试；必须按整序列、生成条件/seed与机器人分组，另外固定测试集。该数据全为合成，尚未独立跑其验证程序，不能引用发布者的“115测试”作为我们完成的核验。

## 本轮保留的证据

都在 `research/benchmark_evidence/`：官方 ZIP 小包、LICENSE、DOI/API 元数据、ZIP清单、少量真实文本样本和 METRIC 论文。`*_zip_manifest.json` 是本轮读取原始中央目录得到；没有解压/下载完整视频和图像集。后续如采用候选，需要冻结下载日期与哈希、写适配器和独立评价协议，才能叫本项目已建立的 benchmark。
