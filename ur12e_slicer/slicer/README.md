# Slicer 前端

Linux 接手先按 [部署说明](../DEPLOYMENT_LINUX.md)安装。完整启动为 `bash ur12e_slicer/start_linux.sh`，由统一入口启动 ROS mock 与 Linux Slicer 5.12.4；`SLICER_EXECUTABLE` 可覆盖本机完整 Slicer 安装中的 `Slicer` 文件。`--backend-only` 跳过 GUI，但不能据此声称前端已验证。

Windows 兼容入口为 `Open_UR12e_Slicer.cmd` 或 `Start_UR12e_Slicer.ps1`，需要此前部署的 Windows Slicer。它们只负责前端；ROS 2 后端由原 Windows / WSL 统一入口启动，不是 Linux 的安装方法。

面板支持目标位置和朝向设置、三维拖动目标点、规划、执行已规划路径、规划并执行、针尖位置和朝向误差，以及场景、截图和演示帧保存。默认目标只是界面初值，未经 IK 验证；收到状态后可用“以实际针尖为目标（上移 30 mm）”构造附近目标，再由 MoveIt 判断可达性。

## 坐标约定

- Slicer 世界位置为 RAS 毫米；发送协议为 UR `base` 下的米，四元数顺序为 `[x,y,z,w]`。
- 首轮已知模拟关系 `BASE_TO_RAS_MM = I`：`p_RAS_mm = 1000 p_base_m`。两套坐标同轴同原点，仅作仿真联调；没有声称影像与真实机器人已经配准。
- 界面的 Roll/Pitch/Yaw 是固定轴 XYZ，组合为 `Rz(yaw) Ry(pitch) Rx(roll)`。
- 简化工具尖端为 `tool0` 局部 `+Z` 的 120 mm：`T_base_tip = T_base_tool0 Trans(0,0,0.12)`，偏移随工具朝向旋转。目标描述的是针尖，朝向与 `tool0` 一致。
- 针尖误差来自当前界面目标与回传 `tcp_pose`，朝向误差为四元数最短相对转角。若修改了目标，误差会对应修改后的目标；后台保存的误差仍对应最近提交的目标。

## 状态与模型

前端原子写 `runtime/target.json`，读取 `runtime/state.json`；完整协议见 [protocol.md](../protocol.md)。后台须 `ready=true` 且反馈时间在 5 秒内才允许提交；执行按钮还要求当前目标与成功计划一致。后端启动前遗留请求不会当作新请求执行。

机器人与工具的位置从实际回传的 `link_transforms`／`tcp_pose` 更新，无反馈时不显示机器人，也不计算到位误差。心跳中断时保留最后反馈、降低透明度并提示过期；不能据此确认到位。`tcp_path_m` 只在后台提供真实规划轨迹的 FK 采样时绘制。

显示模型来自已固定版本的 UR 官方描述包。UR12e 官方配置引用 `ur10e` 网格，这是上游配置关系；`assets/visual_model.json` 记录各 visual origin。前端直接解析 DAE 中的几何和节点矩阵，并应用 URDF origin 与米到毫米缩放。尚未加载模型时可用实际 TF 构成的简化连杆显示。地面仅供显示尺度，不是规划器障碍物。

## 已验证和未验证

2026-10-21 在 Windows Slicer 5.12.4 中通过 13 项前端检查。检查使用隔离目录下的明确测试数据，包括官方网格、坐标与四元数、请求协议、计划与目标对应、过期提示、场景及帧保存。测试图有 `FRONTEND TEST FIXTURE - NOT ROS FEEDBACK` 水印。它单独不能证明 ROS 2、MoveIt 2、规划成功或仿真闭环。

后续 Windows + WSL 参考运行已完成真实 MoveIt/mock 联动、MR 头部和模拟光学链，见 [本轮成果](../NAVIGATION_BENCHMARK_RESULTS_2026-10-23.md)。Linux 新机器运行 `bash ur12e_slicer/run_navigation_linux.sh` 后查看本机新结果；完整入口包括场景生成和独立进程重开 MRB，不能拿旧截图代替本机 GUI 检查。尚未接入真机或物理光学设备。

头部结果查看器说明见 [NAVIGATION_BENCHMARK_VIEWER.md](NAVIGATION_BENCHMARK_VIEWER.md)。`.cmd` / `.ps1` 只适用于 Windows；Linux 使用统一运行入口生成 / 打开当前结果。

开发时可复查：在仓库根目录运行 `python scripts/slicer_client.py ur12e_slicer/slicer/verify_frontend.py`。该脚本使用独立输出目录，不向真实 `runtime/target.json` 写测试命令；结束后恢复等待真实反馈的前端。`reload_frontend.py` 用于开发重载，不作为用户启动入口。
