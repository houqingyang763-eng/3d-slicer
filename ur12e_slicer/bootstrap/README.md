# 环境准备入口与 Windows / WSL 历史记录

**Linux 部署请从 [Linux 部署、运行与验收](../DEPLOYMENT_LINUX.md) 开始。** 当前支持目标为 Ubuntu 24.04 amd64 桌面系统；普通用户执行 `bash ur12e_slicer/bootstrap/install_linux.sh`，只读核对可加 `--check-only`。该入口准备依赖与固定资源，不启动 ROS 或 Slicer。`--offline` 只恢复缓存并检查现有依赖；`--skip-slicer` 跳过 GUI 资源与依赖。

**下文保留为原作者 Windows + WSL 的安装历史及兼容入口说明。** 原生 Linux 不运行 `.ps1`、不安装 Windows 版 Slicer、不导入 WSL 镜像。当前安装器固定 ROS 源配置包和资源校验值，仅安装请求的软件包及其依赖，不运行全系统 `apt upgrade`；已验证 apt 版本与可复现范围见 [`validated_linux_versions.json`](../assets/validated_linux_versions.json)。

GUI 依赖中的 `fonts-noto-cjk` 用于本项目中文面板与图中文字显示，属于项目需要的字体包；Slicer 官方系统库要求另列于来源清单。

## Windows / WSL 历史路线

选定路线：Windows 保留 3D Slicer；WSL 2 中运行 Ubuntu 24.04、ROS 2 Jazzy、MoveIt 2、UR 官方驱动。Slicer 和 ROS 2 通过项目的 `runtime/target.json`、`runtime/state.json` 交换数据，首轮不依赖跨系统 DDS 发现或 TCP 桥。

## 当前证据与边界

- ROS 官方 Jazzy 提供 Ubuntu 24.04/Noble deb；本项目本次完整交付固定为 amd64，UR 官方推荐二进制包 `ros-jazzy-ur`。
- 已阅读 UR 驱动 **jazzy 分支**：`ur_control.launch.py`、`ur_moveit.launch.py` 的 `ur_type` 均包含 `ur12e`，不是将 UR10e 改名代用。
- 这里提供可执行安装脚本。脚本生成成功、静态审阅通过，不等于机器已经装好；安装和实际规划结果应以 `output/` 内本机日志与验收记录为准。
- 首轮使用 `ros2_control` mock hardware，验证通信、运动规划和关节反馈闭环；不验证真实机器人动力学、硬件延时、真实标定精度或接触行为。

## 安装

Windows 侧的 WSL 安装由 `install_wsl.ps1` 处理。其完成后，优先从 PowerShell 使用续装入口：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File '.\ur12e_slicer\bootstrap\prepare_linux.ps1'
```

续装入口读取 `runtime/wsl_install_status.json`，用 Windows 最近启动时间确认是否已在特性启用后重启；尚未重启则提示一次并退出（退出码 `3010`），不会自动重启或循环等待。重启后重跑同一命令即可。

首次会核对已有离线包 `downloads/ubuntu-24.04-amd64.wsl` 的 SHA256 与 `assets/ubuntu_source.json`，再导入专用发行版 `UR12e-Jazzy`；不重复下载。默认数据目录是 `D:\CodexEnvs\UR12e-Jazzy`，没有 D 盘时使用项目的 `runtime/wsl_distributions/UR12e-Jazzy`。可用 `-InstallDirectory 'E:\CodexEnvs\UR12e-Jazzy'` 覆盖；可用 `-ArchivePath` 指向另存的同一官方离线包。已有同名发行版时复用，不删除或重置它。未注册且非空的数据目录会保留并报错，防止覆盖残留数据。

它通过独立参数调用 `wslpath` 和 `bash`，保留 Windows 路径中的空格，以 root 执行下述安装脚本。状态、Windows 侧过程日志与 Linux 安装日志统一保存在 `runtime/`。完成状态 `ready_for_runtime_validation` 只表示包和接口检查通过，不表示仿真验收通过。

本机实际运行中，WSL 的 NAT／localhost 代理提示会写到 stderr；续装入口保留这些日志，但不会把它们作为 `wslpath` 的路径返回值解析。Linux 直连 GitHub 超时，因此首次安装 ROS 源配置包时，由 Windows 下载 `assets/ros_apt_source.json` 固定的官方发布包并核对官方 SHA256，再通过 `ROS_APT_SOURCE_DEB` 交给 Linux 安装。ROS 软件仍由官方签名 apt 源获取；已有源配置包时复用。

需要手工执行时，在 Ubuntu 内运行：

```bash
bash '/mnt/c/Users/HOU/Desktop/3D  Slicer/ur12e_slicer/bootstrap/setup_ros2.sh'
```

普通 Linux 用户安装系统依赖时需具有 `sudo` 权限。脚本只安装请求的软件包及其依赖，不运行全系统升级，不启动机械臂节点，不添加机器人连接，也不改用户 `.bashrc`。Ubuntu 软件源保持系统当前配置，ROS 源采用固定的官方 `ros2-apt-source` 1.3.0（deb 版本 `1.3.0~noble`）。

脚本可重复执行：已安装的软件包由 apt 处理；已有同版本 `ros2-apt-source` 时保留其托管配置，版本不同则保留现状并报错。默认在 `ur12e_slicer/output/environment/setup-<UTC时间>-<PID>/` 留下安装日志、退出码、包版本清单、UR 启动参数，以及本次获取的源配置包信息。成功条件包括两个已安装的 launch 都含 `ur12e`、关键 ROS 包可发现、Python 消息和 action 接口可导入。它不通过虚构运动来代替规划验收。

如果最小 Ubuntu 镜像的 `/etc/apt/sources.list.d/ubuntu.sources` 缺少 `noble-updates` 或 `noble-backports`，脚本会停止。按 ROS 官方提示检查 `Suites:`（应含 `noble noble-updates noble-backports`），修复后重跑；脚本不会覆盖既有软件源策略。如果 GitHub 或 ROS 官方包源无法连通，先保留失败日志，再确定代理或镜像方案，不静默切换第三方源。

## 依赖清单

| 用途 | 直接安装包 |
|---|---|
| ROS 2、RViz、基础消息与工具 | `ros-jazzy-desktop` |
| 包开发工具 | `ros-dev-tools` |
| MoveIt 2 与规划器 | `ros-jazzy-moveit` |
| UR 驱动、描述、官方 MoveIt 配置 | `ros-jazzy-ur` |
| 控制器检查与 TF 检查 | `ros-jazzy-ros2controlcli`、`ros-jazzy-tf2-tools` |
| 项目 Python 支持 | `python3`、`python3-venv`、`python3-numpy`、`python3-yaml` |

## 官方 mock 启动基线

下面是独立检查官方组件的命令；完整项目入口以项目主说明为准。

终端一：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur12e robot_ip:=127.0.0.1 use_mock_hardware:=true \
  initial_joint_controller:=joint_trajectory_controller launch_rviz:=false
```

终端二：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur12e launch_rviz:=false
```

`robot_ip` 是驱动 launch 的必填参数，这里为 mock 占位；`use_mock_hardware:=true` 是仿真条件。当前 Jazzy 的 MoveIt launch 不接收 `robot_ip` 或 `use_mock_hardware`；它读取驱动侧发布的机器人描述。检查 `ros2 control list_controllers`，应看到普通 `joint_trajectory_controller` 激活和 `joint_state_broadcaster` 发布反馈。

官方 MoveIt 配置同时含 scaled 和普通两个轨迹控制器，默认偏向 scaled。项目执行 action 必须明确选择 `joint_trajectory_controller`；不能只修改驱动一侧的初始控制器便假定规划执行一致。普通控制器是官方 mock 示例采用的路线。

mock 使用系统时间，不应设 `use_sim_time:=true`，因为这里没有 Gazebo `/clock`。默认不安装 Gazebo、Docker 或 URSim；若后续验收需要动力学或控制器程序行为，再引入相应模拟器。

## WSL 注意

WSL 2/WSLg 支持 Linux GUI，RViz 可按需启动；GUI 是否正常仍需本机显示验证。共享文件桥不需要先调整 WSL 镜像网络、防火墙或 ROS DDS 网络。Linux 侧处理的路径应使用 `/mnt/c/...`，含空格时加引号。

实物阶段需重新评估运行主机。UR 官方建议低延迟或实时内核以及控制电脑直连机器人网络；本轮 WSL mock 通过不能证明 WSL 适合真实机器人控制。此安装脚本没有设置实时调度权限、机器人 IP、URCap、真实标定、工具负载或设备安全配置。

## 官方依据

- [ROS 2 Jazzy Ubuntu deb 安装文档源码](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)
- [官方 ROS apt 源配置步骤](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst)
- [UR Jazzy 安装说明](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/jazzy/ur_robot_driver/doc/installation/installation.rst)
- [UR Jazzy 驱动 launch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/jazzy/ur_robot_driver/launch/ur_control.launch.py)
- [UR Jazzy MoveIt launch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/jazzy/ur_moveit_config/launch/ur_moveit.launch.py)
- [UR 官方 mock 运动示例](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/jazzy/ur_robot_driver/doc/usage/move.rst)
- [UR MoveIt 控制器配置](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/jazzy/ur_moveit_config/config/moveit_controllers.yaml)
- [Microsoft WSL GUI 条件](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps)
