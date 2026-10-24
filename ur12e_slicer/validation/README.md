# ROS mock 后台验收

`validate_ros_runtime.py` 只向本项目已经运行的 ROS 2 mock 后台发送命令，不安装软件、不启动后台、不模拟机器人。执行时需要暂停 Slicer 的目标发送，避免两个控制端竞争 `runtime/target.json`。

建议在 WSL 中运行，使验收器与 ROS 使用同一壁钟：

```powershell
wsl.exe -d UR12e-Jazzy -u root -- python3 "/mnt/c/Users/HOU/Desktop/3D  Slicer/ur12e_slicer/validation/validate_ros_runtime.py"
```

脚本本身也可由 Windows Python 运行，但本机曾观察到 WSL 壁钟比 Windows 快约 0.6 s，从而触发其未来时间戳检查。该轮保留为失败记录，不将跨系统时差误当作已通过的验收；在 WSL 重跑时不放宽新鲜度或误差门槛。

默认使用 `ur12e_slicer/runtime/`，每轮结果写入独立的 `output/ros_runtime_validation/<时间_随机标识>/`。退出码 0 表示本轮各项通过，1 表示未通过；`summary.json` 中的 `passed` 才是验收结论。保存 fresh request、完整末态、实际反馈时序及本轮 ROS 事件。

覆盖项：

1. 真实 source、文件时间、关节/TF 新鲜度、六关节及完整连杆显示反馈。
2. 从当前实际针尖向下移动 25 mm 的纯规划；检查关节没有运动，且预览来自 MoveIt FK。本包采用显式弯肘模拟初态；小位移仍须经过实际规划验证。
3. 显式引用上一步缓存计划执行；检查 MoveIt 错误码与 action 成功状态、实际关节变化、独立针尖误差。
4. 拒绝重复执行已消耗计划，保持无运动。
5. 在当前实际位姿附近再执行横向位移、位置与朝向共同变化的目标。
6. 对明显不可达目标获得实际 MoveIt 规划失败，且无执行、无运动。
7. 失败目标之后再完成一个可达目标，确认后台可恢复正常工作。

默认实测门槛为针尖位置误差 ≤2 mm、整体朝向误差 ≤0.03 rad。该门槛用于本轮模拟工程验收；只报实测值，不将 mock 的结果推断为真实机械臂精度或实物标定质量。

若首个附近目标也不可达，应审阅真实 ROS 结果和初始姿态；不得更换为前端插值或者放宽误差后宣称同一验收通过。`validation.lock` 只防止两个本脚本同时运行，Slicer 发送仍须由操作者协调。

`capture_ros_environment.sh <输出目录>` 用于只读保存实际 ROS 节点、服务、action、激活的控制器与硬件组件、运行中的 `robot_description` 和软件包版本。其图发现通过 `rclpy` 显式 spin 3 s；不依赖可能尚未发现完整节点的 CLI 默认列表。`environment_summary.json` 同时核对官方 `mock_components/GenericSystem`、JTC active、MoveIt 规划服务与执行 action，从而为 `state.source` 提供外部旁证。

## 2026-10-21 暂停交接

用户要求暂停并关机，已停止发送目标；ROS 正常停止由主任务负责。尚未完成 Slicer 端最终验收和演示录像，不能将后台验收写成完整闭环交付。

- **后台整轮通过**：`output/ros_runtime_validation/20261021_231842_b163ff/summary.json`，`passed=true`，退出码 0，7 项检查、4 次真实 mock 执行。保存的 requests、states、`observations.jsonl`、`ros_events.jsonl` 均在该目录。
- **4 次执行实测误差**（位置 mm / 整体角 rad）：缓存计划执行 `1.722801 / 0.013151`；横向运动 `1.041264 / 0.020823`；位置与朝向共同变化 `0.652194 / 0.015995`；不可达失败后的可达恢复 `1.796333 / 0.018912`。全部满足原定 ≤2 mm、≤0.03 rad。纯规划、已消耗计划重放拒绝、不可达目标失败三项的观测最大关节运动均为 0 rad。
- **官方 mock / ROS 图旁证**：`output/ros_environment_20261021/environment_summary.json`，`passed=true`；同目录有 `robot_description.txt`、`hardware_components.txt`、`controllers.txt`、`nodes.txt`、`services.txt`、`actions.txt`、`graph.json`、`processes.txt`、`package_versions.tsv`。运行中 URDF 与硬件组件均显示 `mock_components/GenericSystem`，硬件和普通 `joint_trajectory_controller` active，MoveIt 规划服务和执行 action 实际存在；未运行实物硬件插件。
- **保留的先前失败**：`output/ros_runtime_validation/20261021_231709_f52088/summary.json`，Windows 运行的验收器在前两项成功后，因 WSL 时间戳约快 0.6 s 触发未来时间戳检查而中止。该轮仍标记 `passed=false`；随后只将同一脚本改在 WSL 同钟运行，通过门槛完全未改。
- **待修事项**：当前 `planner_id=RRTConnectkConfigDefault` 不在本版官方组配置里，日志警告后回退到实际 `geometric::RRTConnect`。只读 `/query_planner_interface` 返回 OMPL `planner_ids=['ur_manipulator']`。本轮证据实际是默认 RRTConnect；暂停前没有修改配置。恢复后可将请求 planner_id 设空，明确使用官方组默认配置，或另行显式配置一个命名 RRTConnect；需按最终方案重新启动并验证，不能把不存在的配置名写成已经选中的算法。

## 2026-10-22 恢复后的配置旁证

- 已将 `config/bridge.yaml` 与 `node.py` 的 `planner_id` 统一设为空字符串，选择安装版 UR MoveIt 组默认。运行中 `ros2 param get /ur12e_slicer_bridge planner_id` 也确认为空；本轮实际规划日志明确使用 `geometric::RRTConnect`，未再出现 `Cannot find planning configuration` 回退警告。旁证保存在 `output/ros_environment_20261022/planner_summary.json`、`planner_id.txt` 与 `planner_log_excerpt.txt`。MoveIt 另有未填写响应 `planner_id` 字段的提示，不能将其误作无效请求配置的回退警告。
- 只读重新采集实际 ROS 环境：`output/ros_environment_20261022/environment_summary.json`，`passed=true`。官方 `mock_components/GenericSystem`、硬件 active、普通 JTC active、规划服务和执行 action 均通过，实际 URDF 中没有实物硬件插件。此次采集不发送运动目标。
- 退出清理已作局部修正：`stop_simulation.sh` 仍核验 PID、进程启动 ticks、命令和进程组，但只向 launch 主进程发送 SIGINT，由 launch 转发一次，避免原先向整组发送再由 launch 二次转发。桥接 main 同时处理 `ExternalShutdownException`，清理期间忽略再次 SIGINT，并且只在 ROS context 活跃时尝试取消 action。Python AST、Bash 语法及正常返回／键盘中断／外部 context 关闭三类清理顺序检查通过。
- 上述局部检查不等同于实际停止通过。随后实际停止已核对：`file_bridge` 正常退出，任务组全部非僵尸进程及专属 Slicer 窗口全部结束，但 `move_group` 仍有 -11、控制器由 launch 升级 SIGTERM 后以 -15 退出。记录在 `output/lifecycle_validation/final_20261022/`；未把空进程组写成全部组件正常退出。
- 针对 Windows 共享读句柄偶发导致 WSL `os.replace` 返回 `PermissionError`，`atomic_json` 增加 80 ms 有界重试窗口；重试同一完整临时文件，期间保留旧状态，不删除目标、不补造反馈时间、不改 1 s 新鲜度。`output/atomic_write_retry_checks.json` 的三项异常注入通过：暂时锁定后成功、持续锁定到期失败且保留旧状态、无关存储错误不重试。这是写入错误处理验证，不能据此声称消除了 WSL 读侧的所有瞬时 `ENOENT`；已运行进程需重启才加载修复。
- 停止入口现在等待旧 PGID 的全部非僵尸成员实际退出后才返回成功，包括 launch 已退出但子进程仍在的情况；25 s 仍有残留则列出 PID／状态／名称并失败，不另发强杀。无记录或已无活成员返回 0，身份不符拒绝发送信号。`check_stop_wait_contract.py` 以虚拟 `/proc`、时钟与模拟信号检查 7 种情况；`output/stop_wait_contract_checks.json` 全部通过。随后真实停止命令等待约 5.80 秒后成功、确认无残留，冷启动与重复启动复用均已通过；旧目标未重放。组件退出错误仍按上一项保留。
