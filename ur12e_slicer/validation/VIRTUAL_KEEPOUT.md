# UR12e 虚拟禁入区规划对照

在 ROS 2 Jazzy 的 UR12e mock 上做三次只规划对照。固定起始关节角、目标针尖位置与朝向、规划器和时限。球半径取 **35 mm**，目标从冷启动针尖沿 UR `base` 的 +Y 方向平移 **200 mm**。120 mm 模拟针随机器人一同参与碰撞检查。

## 运行

在仓库根目录执行，使用普通 Linux 用户和 [已安装的 mock 环境](../DEPLOYMENT_LINUX.md)。若当前正在运行普通 mock，先用 `stop_linux.sh` 结束该会话，再从冷启动姿态开始。

```bash
bash ur12e_slicer/start_linux.sh --keepout
bash ur12e_slicer/run_keepout_linux.sh
bash ur12e_slicer/stop_linux.sh
```

`--keepout` 为三组统一设置 `ompl.ur_manipulator.longest_valid_segment_fraction=0.00025`。结果记录 MoveIt 实际回报的参数值。没有显示环境时，启动加 `--backend-only`，实验加 `--no-slicer`。实验期间保留目标发送窗口供验收独占。[MoveIt OMPL 碰撞采样说明](https://moveit.picknik.ai/main/doc/examples/ompl_interface/ompl_interface_tutorial.html)

## 三组判据

1. **球只显示。** 用与文件桥共用的针尖约束构造一次 `/plan_kinematic_path` 请求，保留全部关节路点。经 `/compute_fk` 求针尖轨迹，在弧长中点附近的采样处取球心。保存 MoveIt 世界中没有该球的回读。
2. **球参与规划。** 在同一位置和半径下，经 `/apply_planning_scene` 加球、经 `/get_planning_scene` 回读。保存的旧轨迹逐样本重查，首尾完整机器人姿态都须有效，且中途须有明确涉及该球的碰撞接触对。原请求原样提交。若得到新轨迹，逐样本用 `/check_state_validity` 复查完整机器人与附加针。若未取得可用规划，保留真实返回码和原始响应。
3. **目标在球内。** 把同一球移到原目标针尖位置，覆盖整个位置容差球。复查原起点仍有效、原先能到达的终态与球碰撞，然后原样提交第三次请求，核对失败及返回的路点数。

每段关节轨迹都保留原路点，并在线性关节插值中以最大单关节步长 **0.01 rad** 补点。碰撞判定来自 MoveIt 的完整机器人状态有效性服务。关节反馈和桥事件用于核对实验中没有运动。实验结束后按本轮精确 ID 移除球并回读，保留其他碰撞物与附加针。[MoveIt 场景服务](https://moveit.picknik.ai/main/doc/examples/planning_scene_ros_api/planning_scene_ros_api_tutorial.html)

## 输出

每轮结果在 `output/keepout_validation/<本轮标识>/`。`summary.json` 汇总三次规划、碰撞采样、场景回读、关节变化和截图。`shared_plan_request.json` 与三份 `*_response.json` 保存 ROS 请求和响应。`*_samples.json` 保存关节采样、针尖 FK 和接触对。Jazzy 的规划服务可将内部失败统一返回为 `99999`，因此目标阻断同时用旧终态的真实球体接触验证。[Jazzy 规划服务实现](https://github.com/moveit/moveit2/blob/jazzy/moveit_ros/move_group/src/default_capabilities/plan_service_capability.cpp)

三张 Slicer PNG 以相同相机和实际起始姿态复现三组条件。金线表示原规划，青线表示复核通过的新规划，红点标出完整机器人发生碰撞时的针尖采样位置。结果适用于固定 mock 模型和离散关节采样。连续碰撞、运动执行、解剖模型和真机行为需要各自验证。

## 2026-11-18 本机实跑

主轮 `20261118_101313_7f5cec` 在 Windows + WSL 2 的 Ubuntu 24.04、ROS 2 Jazzy、MoveIt 2.12.4 与 Linux Slicer 5.12.4 上完成。MoveIt 日志显示使用 OMPL 的 `geometric::RRTConnect`。

| 条件 | 本轮观测 |
| --- | --- |
| 球只显示 | MoveIt 返回 17 个路点。整臂与模拟针的 37 个采样姿态均有效。 |
| 球纳入碰撞 | 原有 37 个样本中 13 个发生球与模拟针接触，起点与原终态均有效。新轨迹 45 个路点，114 个复核样本均有效。针尖采样弧长由约 201 mm 增至约 410 mm。 |
| 球覆盖目标 | 起点仍有效，原终态触球。MoveIt 返回 `99999` 和 0 个路点，后台日志记录 `GOAL_STATE_INVALID`。 |

本轮收到 2,802 条完整关节消息，最大变化 **0 rad**。球已从 MoveIt 回读中消失，附加针仍在。同视角截图尺寸为 1168 × 834。更细采样启用前的试跑 `20261118_100450_5b7012` 也检出旧路径碰撞，但 MoveIt 的 `ValidateSolution` 判定其候选路径无效。启用后另一轮的 136 个绕行样本也全部有效，采样规划器在两轮生成了不同的绕行轨迹。

另用普通 mock 验证了 `--no-slicer` 的未取得可用绕行规划分支，目标阻断和球体清理通过。普通配置下的文件桥 `plan`、FK 预览和原盒体的增删回读也通过回归。停止后确认本任务进程组无活动残留，启动日志仍记录 `move_group` 退出码 `-11`。
