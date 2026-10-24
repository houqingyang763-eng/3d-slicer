# 导航几何与误差评价（模拟）

这套程序检查“影像目标 → 配准与光学观测 → 机器人目标 → 独立真值评价”是否算对，以及无效观测是否使命令失效。它不依赖 ROS，可在普通 Python + NumPy 或 Slicer Python 中运行。

**这里的误差是人为指定扰动下的模拟结果，不能报告为机械臂、定位仪或平台的实测精度；也不代表通过 ASTM F2554 测试。** 机器人被假设准确执行工具标记 M 的姿态命令。机械误差、柔顺变形、碰撞和真实通信控制不在本程序内。

## 运行与复现

使用带 NumPy 的 Python 3；Linux 交付使用 Ubuntu 24.04 的系统 Python 3.12 和 `python3-numpy`。不需要 Codex bundled Python、ROS 或原作者的本机目录。完整依赖安装见 [Linux 部署说明](../DEPLOYMENT_LINUX.md)。从仓库根目录运行：

```bash
python3 ur12e_slicer/navigation_benchmark/run_offline.py \
  --output "$PWD/ur12e_slicer/output/my_offline_run"

python3 -m unittest discover \
  -s ur12e_slicer/navigation_benchmark -p 'test_*.py' -v
python3 ur12e_slicer/validation/check_navigation_metrics.py
```

预期：各命令退出 0；单元测试 26 项，指标 / 授权检查 10 项；离线运行 72 个目标评价中 42 个有效、30 个预期拒绝，15 项解析 / 真值不变性检查通过。离线报告不是 ROS 规划执行记录。统一离线验收也可用 `bash ur12e_slicer/run_navigation_linux.sh --offline-only`，输出目录由入口打印。

再次运行同一输入：`run_offline.py --output <另一个绝对目录> --fixture <已有fixture.json>`。输出：

| 文件 | 内容 |
|---|---|
| `truth.json` | 在生成任何命令之前保存的独立真值快照，含 SHA256 |
| `fixture.json` | 固定 seed、全部输入矩阵、配准点、留出目标及假设阈值 |
| `run.json` | 每目标的真值/显示误差、姿态矩阵、拒绝原因、解析检查 |
| `cases.csv` | 逐场景逐目标的可汇总数值；无效误差为空，不写成 0 |
| `report.html` | 可独立打开的中文结果表和证据边界 |

报告是离线文件，不调用网络。Slicer 内可以将本仓库加入 `sys.path`，再导入 `run_evaluation`：

```python
import sys
from pathlib import Path
repo = Path('/absolute/path/to/3d-slicer')  # 改为本机克隆目录
sys.path.insert(0, str(repo))
from ur12e_slicer.navigation_benchmark.run_offline import run_evaluation
run = run_evaluation(repo / 'ur12e_slicer/output/my_slicer_offline_run')
```

## 坐标与 API

全部长度单位 **mm**；四元数顺序 **xyzw**；右手旋转；`T_A_B` 将 B 中的点映到 A。`I`=影像，`H`=头部参考架，`C`=相机，`B`=机器人底座，`M`=工具标记，`Tip`=针尖。针纵轴定义为工具姿态的 **+Z**。

```text
目标：       T_B_G   = T_B_C · T_C_H · T_H_I · T_I_G
光学显示：   T_I_Tip = inv(T_H_I) · inv(T_C_H) · T_C_M · T_M_Tip
真实执行：   T_B_M   = T_B_G · inv(估计 T_M_Tip)
独立真值：   T_B_Tip = T_B_M · 真值 T_M_Tip
```

真值与估计用深拷贝分开；每次评价前后核对真值哈希。仿真的真值仍是人为给定几何，不等于实物中的独立外部测量。接实物后，需要另外取得符合测量目的的独立参考。

```python
from ur12e_slicer.navigation_benchmark.core import (
    matrix_from_pose, pose_from_matrix, invert,
    estimated_robot_goal, optical_tip_in_image, pose_errors,
)

calibration = {
    'T_B_C': matrix_B_C, 'T_H_I': matrix_H_I, 'T_M_Tip': matrix_M_Tip,
}
observations = {
    'now_s': 20.0,
    'head': {'matrix': matrix_C_H, 'timestamp_s': 20.0,
             'valid': True, 'clock_domain': 'simulation_seconds'},
    'tool': {'matrix': matrix_C_M, 'timestamp_s': 20.0,
             'valid': True, 'clock_domain': 'simulation_seconds'},
}
goal_B_mm = estimated_robot_goal(calibration, observations, target_I)
display_tip_I = optical_tip_in_image(calibration, observations)
errors = pose_errors(actual_tip_I, target_I)
# errors: position_error_mm / axis_error_deg / orientation_error_deg
position_mm, quaternion_xyzw = pose_from_matrix(goal_B_mm)
```

`estimated_robot_goal` 只需要头部观测来计算几何目标；完整演示执行前必须调用 `validate_observations` 检查头部和工具，或使用同目录 `gateway.NavigationGate`。有工具遮挡时，仅仅能算出目标不表示允许执行。

`validate_observations` 必须取得显式 `now_s`，它与两路观测时间戳使用同一时钟域。默认假设为最大帧龄 100 ms、最大两路时间差 30 ms；这些只是故障注入参数，不是传感器规格或硬件安全阈值。不外推、不用零矩阵填补缺帧。无效、过期、未来、跨对象不同步或非刚体矩阵抛出 `ObservationRejected`，含可读 `code`。

`validate_execution_snapshot` 检查头模姿态变化是否使旧命令失效；本离线 fixture 使用 0.5 mm / 0.5° 的假设阈值。它不提供硬件停止功能。接入真实机器人时，也必须处理目标版本、标定版本、控制器与轨迹生命周期，不能只调用几何函数。

应用层 `gateway.NavigationGate` 已统一使用0.5 mm／0.5°，并从授权端 `clock` 获取当前时刻，不信任旧观测包中的 `now_s`。默认时钟域为 `unix_seconds`；WSL 集成明确指定 `wsl_wall_simulation`。离线纯函数中的 `now_s` 是调用者注入的模拟时钟，不能直接用作实物授权时钟。旧包重放与跨对象时间不齐均有独立检查。

静态点数据适配器：`validation/evaluate_recorded_points.py --input <JSON> --output <JSON>`，输入示例见 `research/benchmark_evidence/astm_audit/analytic_input_example.json`。`metrics.py`区分bias、平均距离、RMS、固定点precision及span；多目标仅输出误差向量离散度，不叫固定点重复性。此适配器只做统计，不能代替标准采集协议。

## 测试场景与解释

固定 seed 为 `20261023`，使用非单位旋转和平移；8 个拟合点与 6 个留出目标互不重合。仅做 SE(3) 刚体拟合，不允许缩放。FRE 来自拟合点，针尖目标误差来自留出点，两者分别报告。

12 个场景共 72 个目标评价：无扰动、一个配准点偏差、全部配准点同向偏差、底座外参平移偏差、工具偏差、头模移动后重算、头模移动后旧命令失效、工具遮挡、头部遮挡、同样过期的两路帧、跨对象时间不齐、已知时延动态运动。

2026-10-23 本轮运行：42 个有效、30 个拒绝，15 项解析/真值不变性检查通过；26 个数学 oracle 与失效处理单元测试通过。关键结果：

| 人为指定条件 | 独立几何真值误差 | 显示误差 | 说明 |
|---|---:|---:|---|
| 所有配准点同向偏移 2.5 mm | 2.5 mm | 约 0 | 同一错误标定用于命令与显示，可能自洽却不正确；FRE 也可约 0 |
| 工具标定偏差 2.5 mm / 3° | 2.5 mm / 3° | 约 0 | 工具偏置不能由复用同一标定的显示证明正确 |
| 相机到底座外参平移偏差 3 mm | 3 mm | 3 mm | 外部光学读数可揭示此项偏差 |
| 头模 20 mm/s，延迟 80 ms | 1.6 mm | 约 0 | 解析值为速度 × 延迟；两路仍可能同样过时且未超帧龄阈值 |
| 遮挡、过期、时间不齐、头模移动后旧命令 | 不报告 | 不报告 | 拒绝，不记作零误差，也不执行命令 |

这几个数值的目的是验证评价程序能发现已知问题。固定扰动量不代表现实中各误差项的大小或重要性排序。

## 如何扩展

可以替换 fixture 的影像点、头模坐标和姿态，但拟合点、留出点和真值来源必须保持分离。真实光学数据需由驱动提供有效位、时间戳与坐标约定；ROS m 与本模块 mm 的换算只能放在明确的接口边界。

新算法应在同一数据、同一留出规则和同一测量定义上比较。此处六个模拟目标和人为偏差并不是权威 benchmark，不能替代实体测试体、独立计量、重复试验及与论文一致的实验协议。
