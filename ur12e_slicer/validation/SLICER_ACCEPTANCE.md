# Slicer 实际闭环验收与录像

本验收在真实 Windows Slicer 5.12.4 的 Qt 主线程中运行。后台必须是本任务实际 ROS 2／MoveIt 2 mock hardware；验收不会写入模拟反馈、做机械臂前端插值，或连接实物。

先启动本任务的 `slicer/Open_UR12e_Slicer.cmd` 和 ROS 后台。自动化队列独立位于 `runtime/slicer_bridge/`，不会选择项目旧有的课堂 Slicer 会话。

```powershell
python .\ur12e_slicer\slicer\task_client.py .\ur12e_slicer\validation\prepare_slicer_acceptance.py
```

准备命令只加载验收脚本，不发送目标。确认后台验收脚本已经结束、没有其他控制端发送目标后，再执行：

```powershell
python .\ur12e_slicer\slicer\task_client.py .\ur12e_slicer\validation\start_slicer_acceptance.py
```

默认目标从当时真实针尖反馈出发，沿机器人 base 的 Z 方向下移 25 mm，并把固定轴 yaw 增加 3°。Slicer 控件实际采用的数值、换算后的完整目标位姿均写入结果。该小范围目标仍须通过真实 MoveIt 规划；不预设可达。

流程为：设置 Slicer 控件 → 停留展示目标 → 通过面板的规划入口发送 → 等待实际 MoveIt FK 预览 → 通过面板执行缓存计划 → 等待 MoveIt 成功及反馈稳定 → 保存结果。Qt 定时器推进流程，界面与 5 fps 截图记录持续运行。

查看 `output/slicer_acceptance_status.json`。`stage=complete` 且独立运行目录 `results.json` 的 `all_passed=true` 才表示通过。验收同时检查真实 ROS 来源、新鲜反馈、真实规划/执行返回码、观察到的关节运动、规划预览、Slicer TCP 显示变换及独立位姿误差。执行完成后还核对至少 4 秒的持续反馈：至少 10 次不同的 ROS 状态更新，整段位置误差 ≤2 mm、整体旋转角 ≤0.03 rad，且始终对应本次成功执行。它不是实物精度或临床误差要求。

每轮产物位于 `output/slicer_acceptance/run_*/`：

- 三张目标、规划预览与执行完成截图。
- `UR12e_closed_loop.mrb`：含官方机器人模型、目标、实际工具及规划路径的 Slicer 场景。
- `results.json`、`observations.json`：请求、完整反馈、检查结果和时间序列。
- `frames/`：真实 Slicer 窗口 PNG 与逐帧反馈、时间戳清单。

用本任务的媒体环境编码视频：

```powershell
.\ur12e_slicer\runtime\media_env\Scripts\python.exe .\ur12e_slicer\validation\encode_slicer_recording.py '<本轮frames完整路径>'
```

录帧从实际 Slicer 主窗口抓取，在 Qt 主线程按比例缩到宽度至多 1600 px，采用低压缩无损 PNG 避免压缩阻塞反馈。每帧记录原窗口尺寸、保存尺寸、原始抓取时戳、抓取耗时和当时实际反馈。编码使用实际帧时间戳生成可变帧时长 MP4，保留运行时长，不合成机器人运动。视频、编码日志和 `video_manifest.json` 与本轮结果相邻。

`benchmark_slicer_capture.py` 与 `profile_slicer_recording.py` 只测实际截图与 Qt 定时器性能，不发送机器人目标。2026-10-22 优化后实测 4.594 秒保存 21 帧、观察到 38 次不同 ROS 反馈，最长单帧耗时 0.183 秒，见 `output/capture_benchmark/timer_profile.json`。优化前的首轮 `run_20261022_163634` 因原尺寸 PNG 压缩阻塞而未通过录帧数和稳定反馈更新数检查；其数值动作检查通过，但整轮仍保留为失败。

## 2026-10-22 最终实际结果

`output/slicer_acceptance/run_20261022_165004/results.json`：21 项全部通过，位置 1.205062 mm、整体角 0.019561412 rad，保持 4.062 秒，20 次不同反馈更新，关节观测漂移 0。48 张真实窗口帧已编码为同目录 MP4，完整解码通过；最终截图和 MRB 均已保存。另一次通过结果 `run_20261022_163931` 与首轮未通过结果继续保留。

最终面板的反馈中断／恢复另有 9 项实际检查通过，见 `output/feedback_disconnect/20261022_165114_5ce94f/results.json`；停止后的冷启动与重复启动复用见 `output/lifecycle_validation/final_20261022/restart_and_reuse.json`。界面能够发送和显示反馈不等于退出组件全部正常；退出限制见主 `RESULTS.md`。

## 2026-10-21 暂停交接（历史）

用户要求暂停并准备关机。Slicer 侧已完成任务专属自动化队列隔离、真实 ROS TF 的官方 UR 模型与简化工具显示核对，以及非阻塞验收/录帧/编码脚本准备。实际 Slicer 已读取 `ros2_mock_hardware`、六关节与完整工具位姿，截图目视正常。

**Slicer 目标发送、完整 Slicer 闭环验收及演示录像尚未执行。** 本前端会话的 `last_frontend_request` 仍为 `null`；不能把后台独立验收成功或当前实时显示称为 Slicer 发起的完整闭环。

暂停检查点位于 `output/paused_20261021/`：`UR12e_paused_checkpoint.mrb`、`slicer_paused_checkpoint.png` 与 `checkpoint.json`。场景保存返回成功，checkpoint 保留实际 ROS 末态、当前未发送的 Slicer 目标和来源信息。`output/slicer_acceptance_status.json` 已记为 `paused_without_start`。随后通过 Qt 正常退出本任务 Slicer PID 12612，退出已检查；没有强制结束进程或操作其他 Slicer 窗口。

恢复时先由整体任务确认 ROS 后台与独占目标发送窗口，再用上述 prepare/start 入口重新开始 Slicer 验收。Windows 与 WSL 当时存在约 0.6 s 壁钟偏差；前端 freshness 上界未将其误判为断线，录帧时长只使用 Windows 单钟时间戳。
