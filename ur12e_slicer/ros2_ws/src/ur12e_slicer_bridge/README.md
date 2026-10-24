# UR12e Slicer ROS 2 backend

本包在 Ubuntu 24.04 / ROS 2 Jazzy 中运行，使用 UR 官方驱动的 mock hardware、MoveIt 2 现成规划服务和执行 action。Linux Slicer 与 ROS 通过同一项目目录内的 JSON 文件桥接；Windows + WSL 是保留的历史兼容环境。完整约定见上层 `ur12e_slicer/protocol.md`。

## 构建与启动

系统依赖由项目统一 bootstrap 安装，本包不自行安装。必须具备 Jazzy 的 `ur_robot_driver`、`ur_description`、`ur_moveit_config`、MoveIt 2、rclpy、tf2_ros 和 colcon。若 apt 版本没有 `ur12e`，应先更新/选择包含 UR12e 的官方 Jazzy 版本，不能用 UR10e 名称代替 UR12e。

新机器先按 [Linux 部署说明](../../../DEPLOYMENT_LINUX.md)安装，从仓库根目录启动：

```bash
bash ur12e_slicer/bootstrap/install_linux.sh
bash ur12e_slicer/start_linux.sh --backend-only
bash ur12e_slicer/status_linux.sh
```

统一入口完成 source、普通用户可写的独立 colcon 构建目录和进程登记。`UR12E_BUILD_DIR` 可覆盖构建目录；默认在当前用户的 XDG / HOME cache 中，不能依赖 `/root`。结束时运行 `bash ur12e_slicer/stop_linux.sh --backend-only`。GUI / 完整导航验证用不带 `--backend-only` 的整体启动及 `run_navigation_linux.sh`。

`mock.launch.py` 固定 `ur_type=ur12e`、`use_mock_hardware=true`、`robot_ip=127.0.0.1`，关闭 dashboard client 和工具通信，不暴露切换实物的参数。普通 `joint_trajectory_controller` 被激活，MoveIt 执行 goal 也明确选择它；不能只改驱动控制器而让 MoveIt 继续使用默认 scaled controller。

ROS mock hardware 不使用 Gazebo 或 `/clock`，因此 `use_sim_time=false`。官方 MoveIt launch 本身等待机器人描述；桥接节点等待关节、TF 与服务，避免用固定 sleep 猜测系统就绪。命令准备最多等待 20 s；规划响应最多等待规划预算加 15 s；执行默认 120 s 超时并请求取消。错误写入 `state.message` 与日志。

`config/bridge.yaml` 是针尖偏移、规划组/算法、误差容差和速度倍率的集中机器人侧配置。针是沿 tool0 +Z 的简化圆柱，当前不包含真实工具外形、组织模型或真实标定。RRTConnect 为首轮默认，不声称算法最优。

## 本机核查范围

纯协议测试不依赖 ROS。从仓库根目录：

```bash
PYTHONPATH=ur12e_slicer/ros2_ws/src/ur12e_slicer_bridge \
  python3 -m unittest discover \
  -s ur12e_slicer/ros2_ws/src/ur12e_slicer_bridge/tests -v
python3 -m compileall -q ur12e_slicer/ros2_ws/src/ur12e_slicer_bridge
```

这些检查仅覆盖 JSON 契约、针尖方向换算、四元数误差与原子写入。2026-10-21 至 23 的 Windows + WSL 参考运行已完成真实 ROS 规划和 mock 执行，见 [本轮成果](../../../NAVIGATION_BENCHMARK_RESULTS_2026-10-23.md)。接收方仍需执行本机验收：`bash ur12e_slicer/run_navigation_linux.sh --no-slicer` 可验证 ROS 与导航链；无该实际返回记录时不能仅凭协议测试或旧报告宣称本机闭环通过。

## 官方源码核对（2026-10-21）

核对的是官方仓库 **jazzy 分支当前源码**，不是保证任意旧 apt 包相同。现场安装后仍需 `ros2 launch ... --show-args` 核对。

- [UR Jazzy driver launch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/jazzy/ur_robot_driver/launch/ur_control.launch.py)：`ur12e` 在支持列表中；mock 参数为 `use_mock_hardware`；默认是 scaled JTC，本包明确改用 ordinary JTC。
- [UR Jazzy MoveIt launch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/jazzy/ur_moveit_config/launch/ur_moveit.launch.py)：`ur_type=ur12e`；不向该 launch 传不存在的 `robot_ip` 或 `use_mock_hardware`。
- [MoveIt controller config](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/jazzy/ur_moveit_config/config/moveit_controllers.yaml)：同时列出 scaled 和 ordinary JTC，scaled 为默认，需在 ExecuteTrajectory 明确选择 ordinary。
- [GetMotionPlan](https://github.com/moveit/moveit_msgs/blob/ros2/srv/GetMotionPlan.srv)、[MotionPlanRequest](https://github.com/moveit/moveit_msgs/blob/ros2/msg/MotionPlanRequest.msg)、[ExecuteTrajectory](https://github.com/moveit/moveit_msgs/blob/ros2/action/ExecuteTrajectory.action)：服务/action 字段对应本包现成接口调用。
