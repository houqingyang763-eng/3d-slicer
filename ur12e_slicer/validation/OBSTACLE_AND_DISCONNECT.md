# 静态障碍物与反馈中断验收

这两项使用已经运行的本任务 ROS 2／MoveIt 2 mock hardware 和真实 Windows Slicer，不写入伪造的机器人反馈。运行期间需要独占目标发送窗口；不要同时操作面板发送目标或运行其他验收器。

## 运行入口

在专用 `UR12e-Jazzy` 的 Bash 中执行（项目路径保留两个空格）：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=62 ROS_LOCALHOST_ONLY=1
cd "/mnt/c/Users/HOU/Desktop/3D  Slicer/ur12e_slicer"
python3 validation/validate_obstacle_scene.py --slicer
python3 validation/validate_feedback_disconnect.py
```

机器人侧已有 `ready=true`、六关节、TF、附加针碰撞体和 MoveIt 服务时才会开始。两者共用 `runtime/validation.lock`；异常也会释放自己创建的锁。不要删去他人运行中的锁。

## 障碍场景验证的内容

`scene_tool.py` 通过 `/apply_planning_scene` 增删带 `ur12e_demo_` 前缀的盒体，再用 `/get_planning_scene` 回读。盒体的原始回读坐标可能是 `world`，工具通过真实 TF 转成机器人 `base`；不能直接把回读坐标当成 Slicer RAS。

`runtime/obstacle_scene.json` 只保存成功回读的结果。`obstacle_display.py` 在 Slicer 中将其转成毫米和既定的 base→RAS 模拟变换，核对模型尺寸与显示变换。它是场景回读快照，当前不会持续订阅外部场景变化。单独刷新可运行 `scene_tool.py inspect`，再用 `task_client.py validation/show_obstacle_scene.py`。

自动验收顺序：

1. 在没有本工具盒体时，仅规划从实测针尖下移 25 mm 的目标，核对不运动。
2. 放置 60×60×120 mm 的侧方盒体，核对实际起点与已求得终态仍有效；重新规划并执行。
3. 对实际关节规划轨迹做插值，最大关节步长 0.03 rad，逐样本调用真实 `/check_state_validity`。这是离散采样核验，不能写成连续碰撞证明。
4. 另选实测针尖下移 50 mm 的目标，先证明无目标盒时可规划，再把 40 mm 盒体放到目标针尖处。
5. 用真实碰撞服务核对当前姿态仍有效、先前已求得终态与目标盒碰撞；`plan_execute` 必须实际规划失败且机器人关节不变。
6. 移除目标盒，再规划同一目标，核对恢复。

默认 finally 会移除这两个精确 ID 的盒体，回读确认并同步清除 Slicer 显示；不改其他场景对象。`--keep-side-box` 可保留侧方盒，目标盒始终移除。Slicer 验收会保存侧障执行、目标阻断和恢复的截图／场景；不能将侧方可达运动表述为已演示绕障。

## 反馈中断验证的内容

`validate_feedback_disconnect.py` 先校验 `linux_launch.json` 的 PID、进程启动时钟和进程组，再从同一组中定位唯一的本任务 `file_bridge`。它只对该进程发 `SIGSTOP`，暂停真实状态文件更新约 8 秒；不停止控制器，也不改写 `state.json`。

独立 ROS 订阅继续接收真实 `/joint_states`。Slicer 应显示“反馈已中断”、把机械臂淡化、禁用全部目标命令；直接调用面板发送入口也应被拒绝且 `target.json` 内容与更新时间不变。finally 校验同一 PID 身份后发 `SIGCONT`，等待同一桥的新鲜反馈，并核对面板重新启用规划。

这证明文件桥反馈停更时的界面行为，不是网线拔除、实际机械臂断电或控制器急停测试。界面的时间显示也不构成跨 Windows／WSL 的端到端延迟测量。

## 2026-10-22 实际结果

- 障碍验收：`output/obstacle_validation/20261022_164147_c6cf4d/summary.json`，6 项全部通过，15.15 s。侧障执行针尖误差 1.874127 mm、整体旋转误差 0.020828885 rad；真实轨迹 7 个插值采样全部有效。目标盒场景下当前姿态有效，目标终态碰撞对象明确为 `ur12e_demo_goal_blocker` 与 `slicer_simplified_needle`；规划失败时最大关节变化为 0。移除后规划恢复。
- Slicer 回读显示：4 次场景截图的盒体尺寸差与变换差均为 0；补存 `UR12e_obstacle_scene.mrb` 成功，文件位于同一成功目录。该场景含侧障与恢复后的计划，仍是 mock 仿真。
- 反馈中断：`output/feedback_disconnect/20261022_164210_bc1a74/results.json`，7 项检查全部通过，3 张实际界面截图；同一 PID 已恢复，独立 ROS 关节反馈始终持续。
- 最终代码复验：`output/feedback_disconnect/20261022_165114_5ce94f/results.json`，9 项检查全部通过。除既有 7 项外，新增核对误差文字为“末次反馈（已过期）”、针尖位置文字为“末次针尖 RAS（反馈无效）”；恢复后两种失效文字消失。暂停期独立 ROS 订阅收到 4005 条关节消息，同一桥 PID 498 已恢复，目标文件未变化，未重复障碍验收或下发实际目标。
- 最终清理：两个本工具盒体均已从实际 MoveIt 场景移除，附加针仍保留；Slicer 同步确认 0 个本工具盒体，成功目录的 `post_validation_cleanup_scene.json` 和 `09_scene_cleaned.png` 保留证据。

保留此前两次未通过的记录：`20261022_164110_822615` 暴露“删除不存在对象”被 MoveIt 拒绝，工具已改为先回读；`20261022_164119_f87a8d` 在 WSL 读取共享文件时出现瞬间 ENOENT，该轮没有执行运动。独立障碍验证器对 ENOENT/EACCES 最多重读 0.25 s，仍执行原有 1 s 新鲜度检查；成功轮只需重读 1 次。未修改通用机器人验收器的历史契约。
