# Slicer ↔ ROS 2 共享文件协议 v1

Windows Slicer 与 WSL 中的 ROS 2 访问同一个 `ur12e_slicer/runtime/`。两端分别先写同目录临时文件，再用原子替换更新 `target.json` 和 `state.json`；读取方不得把临时文件当作消息。此桥用于本项目的 mock hardware 阶段，执行状态必须来自真实 ROS 2/MoveIt 接口，不能用前端动画或插值代替。

## 统一坐标与工具

- 所有位置使用 **UR `base` 坐标系、米**；关节角与朝向误差使用弧度。
- 四元数排列为 **`[x,y,z,w]`**，描述物体局部坐标到 `base` 的旋转。它不是欧拉角或 UR 旋转向量。
- 用户下发的目标和反馈中的 `tcp_pose` 均表示**简化针尖 `needle_tip`**。
- 针尖与 `tool0` 朝向相同，默认在 `tool0` **局部 +Z 方向 0.12 m**。唯一机器人侧配置为 `config/bridge.yaml:tip_offset_m`，前端必须读取 `state.tool.tip_offset_m`。
- 机器人规划约束以 `tool0` 为 link，通过 `PositionConstraint.target_point_offset` 指向其局部针尖 `[0,0,0.12]`，位置球直接约束目标针尖；朝向约束仍施加于 `tool0`。名义关系为 `p_tool0 = p_tip - R(q) · offset`，但不以目标朝向换算后固定约束法兰，以免姿态容差额外放大针尖位置误差。MoveIt 场景中附加同长度、半径 3 mm 的简化圆柱针，用于该模拟工具的碰撞检查。
- `base` 和 `base_link` 不应直接当作同一坐标；各连杆反馈均通过 TF 查询到 `base`。
- Slicer 的毫米与 RAS 变换由前端显式处理。本阶段采用记录清楚的模拟影像—机器人变换，不构成实物标定。

## target.json：前端写，机器人侧读

```json
{
  "schema_version": 1,
  "request_id": "unique-command-id",
  "command": "plan_execute",
  "frame_id": "base",
  "position_m": [0.45, -0.35, 0.45],
  "orientation_xyzw": [1.0, 0.0, 0.0, 0.0],
  "timestamp": 1792530000.0
}
```

上例仅说明字段格式，位置**尚未经过本机 ROS 逆解验证**；时间戳必须在提交时重新产生。本包 mock 初态显式采用 `config/mock_initial_positions.yaml` 的弯肘配置；通过本包描述 launch 向官方 UR xacro 指定该文件。运行后应先查看实时 `tcp_pose`，优先尝试下移 0.025 m；小范围位移也必须经过本机 ROS 规划验证，不预先宣称可达。

| 字段 | 约定 |
|---|---|
| `schema_version` | 整数 `1` |
| `request_id` | 每次提交唯一、非空、最多 128 字符；不要复用 |
| `command` | `plan` 仅规划；`plan_execute` 规划成功后执行；`execute` 执行已缓存计划 |
| `frame_id` | 固定 `base`；错误坐标系直接拒绝 |
| `position_m` | 三个有限数字，目标针尖位置 |
| `orientation_xyzw` | 四个有限数字，非零四元数；后台归一化 |
| `timestamp` | Unix 秒，允许小数；默认接受过去 60 秒以内、未来不超过 10 秒 |
| `plan_request_id` | `execute` 必填，引用成功 `plan` 的 `request_id`；目标位姿必须与缓存计划一致 |

同一时间只有一个命令在执行。忙时收到新请求会拒绝并在反馈 `last_rejected_request_id` 及 `ros_events.jsonl` 留痕；不会中途切换目标。缓存计划执行前会检查关节相对轨迹起点是否变化超过 0.01 rad；执行成功后消耗该计划，不能重复执行。

后台启动会**忽略当时已经存在的 target 文件及其中的 request_id，不删除文件**。前端见到 `ready=true` 后应重新生成 request_id 和时间戳再提交。后台重启不是继续执行遗留命令的信号。

## state.json：机器人侧写，前端读

```json
{
  "schema_version": 1,
  "source": "ros2_mock_hardware",
  "timestamp": 1792530001.2,
  "request_id": "unique-command-id",
  "status": "executing",
  "message": "Executing trajectory through MoveIt on mock hardware",
  "ready": true,
  "feedback_valid": true,
  "joint_names": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
  "joint_positions_rad": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
  "joint_state_age_s": 0.02,
  "tf_age_s": 0.02,
  "link_transforms": {
    "tool0": {"position_m": [0.45, -0.35, 0.57], "orientation_xyzw": [1.0, 0.0, 0.0, 0.0]},
    "needle_tip": {"position_m": [0.45, -0.35, 0.45], "orientation_xyzw": [1.0, 0.0, 0.0, 0.0]}
  },
  "tcp_pose": {"frame_id": "base", "position_m": [0.45, -0.35, 0.45], "orientation_xyzw": [1.0, 0.0, 0.0, 0.0]},
  "target_pose": {"frame_id": "base", "position_m": [0.45, -0.35, 0.45], "orientation_xyzw": [1.0, 0.0, 0.0, 0.0]},
  "position_error_m": 0.0,
  "orientation_error_rad": 0.0,
  "needle_collision_ready": true,
  "plan": {"status": "succeeded", "request_id": "unique-command-id", "point_count": 42, "tcp_path_m": []},
  "execution": {"status": "executing"},
  "tool": {"parent_frame": "tool0", "tip_offset_m": [0.0, 0.0, 0.12]}
}
```

这是**示意结构，不是执行结果**。初始字段可为 `null`/空列表；误差只有反馈新鲜且有目标时才计算。没有 TF 的连杆不出现在 `link_transforms`，不得补成零位姿。前端应以文件更新时间/`timestamp` 判断整个后台是否在线，不能仅依赖内部 `feedback_valid`。

`link_transforms` 的键：`base_link`、`shoulder_link`、`upper_arm_link`、`forearm_link`、`wrist_1_link`、`wrist_2_link`、`wrist_3_link`、`tool0`、`needle_tip`；所有变换均是 `base → link`。其位置和四元数是该 link 在 base 中的位姿，可直接作为网格父变换。

`status`：`waiting_for_robot`、`idle`、`planning`、`planned`、`executing`、`succeeded`、`error`。`message` 给出原因。`ready` 要求关节/TF 新鲜、针碰撞体就绪、规划服务及执行 action 可用；不会证明任意目标都可达。`succeeded` 表示 MoveIt action 与错误码均返回成功，仍须独立核对末端误差。

`plan` 可增补 `moveit_error_code`、`planning_time_s`、`joint_trajectory:{joint_names,points:[{positions_rad,time_from_start_s}]}`。`tcp_path_m` 是实际关节规划轨迹经 MoveIt `/compute_fk` 求得的最多 24 个针尖采样点；它尚未成功返回时保持空列表，`preview_message` 给出原因，前端不可画直线假装规划。`execution` 可增补 `moveit_error_code` 与 action 的 `action_status`。

默认规划约束为 `tool0` 上针尖目标点的位置球半径 0.002 m，**整体朝向角预算 0.03 rad**。`orientation_tolerance_rad` 表示该整体预算；使用旋转向量参数化时，三个轴分别分配 `0.03/√3 ≈ 0.0173205 rad`，使合成角不超过整体预算。反馈与独立验收的 `orientation_error_rad` 均使用整体最小旋转角 `2 acos(|q_actual·q_target|)`，不是任一分轴误差。所有数值只是仿真工程配置，未确认为实物验收要求。

## ROS 接口和证据

- `/joint_states`：关节实际模拟反馈；`/tf`、`/tf_static`：各 link 到 base 的变换。
- `/plan_kinematic_path`：`moveit_msgs/srv/GetMotionPlan`。
- `/execute_trajectory`：`moveit_msgs/action/ExecuteTrajectory`，显式 `controller_names=["joint_trajectory_controller"]`，与 mock 驱动的普通 JTC 一致。
- `/apply_planning_scene`：附加简化针碰撞几何；`/compute_fk`：真实规划预览采样。
- `runtime/ros_events.jsonl`：追加记录接收/拒绝、规划返回、执行返回与当时末端误差；不能把纯协议测试日志当成 ROS 执行证据。

启动时指定独立 `ROS_DOMAIN_ID=62`、`ROS_LOCALHOST_ONLY=1`。本阶段只支持随包提供的固定 mock launch，不将本桥接到真实机械臂或实验室既有控制网络。
