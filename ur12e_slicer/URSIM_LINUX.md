# URSim 虚拟控制柜：安装、基础控制与 Slicer

更新日期：2026-10-31。目标为 Ubuntu 24.04 amd64、普通用户、本机 UR12e 仿真。先按 [Linux 部署说明](DEPLOYMENT_LINUX.md)准备系统 Python 3.12、ROS 2 Jazzy / MoveIt、UR 官方驱动及 Linux Slicer。本路线额外使用 Docker；原 mock 入口保留，不要求默认用户安装 Docker。

最新复跑：10 月 31 日两轮基础控制均 5/5，Slicer 均 20/21；录帧 19<20 导致外层失败。10 月 27 日完整通过保留为历史记录，不能作为最新复跑全通过。头模仅恢复会话显示，默认入口仍只加载机器人，未完成整套 URSim 头部导航评价。见 [最新上下文](../PROJECT_CONTEXT.md)和 [精选证据](reference_results/2026-10-31-ursim/README.md)。

## 1. 当前状态与版本依据

用户已确认继续使用 ROS 2 / MoveIt 连接 URSim，真机安装 External Control 1.0.5。本次固定如下资源，详见 [来源清单](assets/ursim_source.json)：

| 项目 | 固定选择 |
|---|---|
| 官方镜像 | `universalrobots/ursim_e-series:5.23.0`，Linux amd64 |
| 镜像 digest | `sha256:f1211a7feb4252135b1439f603f4e17ff826b4b71574f3963c868efe0498b4a9`；实际运行按 digest 引用 |
| 机器人型号 | ROS `ur_type=ur12e`；Docker `ROBOT_MODEL=UR12` |
| External Control | 1.0.5，官方 `.jar`，35300 bytes |
| URCap SHA256 | `31a59bd939c339a8ce4f2912d7b51299d81469608ba64a070697b105ea18e0a3` |
| 本机虚拟控制柜 | `192.168.62.2`，内部桥 `192.168.62.0/24` |
| 驱动回连主机 | `192.168.62.1`，External Control script sender 端口 `50002` |
| ROS 范围 | `ROS_DOMAIN_ID=62`、`ROS_LOCALHOST_ONLY=1`、系统时间 |

UR12e 到 `UR12` 的映射来自 [UR 官方启动脚本](https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/71fa9ba88b7a036b5a6d2f73909a8bfb81fa07a9/scripts/start_ursim.sh)；该脚本对 UR12e 的 PolyScope 5 最低版本检查为 5.22.0。URCap 来自 [官方 v1.0.5 发布](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/tag/v1.0.5)，上述 SHA256 为本次从官方资产计算并匹配的值，旧发布没有独立 digest 字段。照片中的 Baseline / PolyScope / Controller / Firmware 组件号与此 URSim 包的准确对应尚未确定；当前版本选择提供 UR12e 支持，不宣称精确复刻真机软件。

本机 Windows + WSL2 / WSLg 已实际完成 Docker / URSim、External Control、ROS scaled controller 与 Linux Slicer 联动，相关离线回归通过。完整轮 **`20261027T044855Z_f8a2ba`** 执行 `run_ursim_linux.sh --stop-after` 退出 0：基础控制 5 项、Slicer 21 项和验收停止无残留检查通过，详见 [本轮结果与过程](URSIM_RESULTS_2026-10-27.md)。停止检查时 backend / Slicer / GUI relay / 容器均不运行；后续演示可再次启动，当前状态以 status 为准。误差来自虚拟模型关节 / TF，不是实物精度。早期共享文件、时钟和归档失败记录保留；10 月 24 日 mock 参考结果不移作本轮 URSim 结果。

## 2. 安装与资源准备

在仓库根目录执行：

```bash
bash ur12e_slicer/bootstrap/install_ursim.sh
```

以普通用户运行，只有系统安装步骤使用 sudo。若当前没有 Docker，入口安装发行版 `docker.io`、启动服务并加入 docker 组；组成员变更后重新打开 Linux 登录会话，再重跑同一入口。只有本用户能够访问本机 `/var/run/docker.sock` 后才继续准备资源。已有 Docker 时复用本机服务；入口不改用远程 Docker context。

`prepare_ursim.py` 按清单下载 URCap / 小型 External Control 程序模板、核对 SHA256，并拉取固定 digest 镜像、检查 RepoDigests。下载暂存位于 `downloads/ursim/`；程序与 URCap 位于当前用户 XDG cache 下、按项目路径哈希隔离的 `ur12e_slicer/<哈希>/ursim/`。这些文件和容器层均不提交。

URSim 高频控制文件位于该原生 Linux cache 的 **`bridge/`**：`state.json`、`target.json`、`validation.lock` 和 `ros_events.jsonl`。基础验收原始日志位于同级 **`validation/<轮次>/`**，避免高频读写穿过 WSL DrvFS。项目 `runtime/` 仅保存该模式的管理所有权、Slicer 心跳和任务桥队列。状态命令输出 `control_runtime`；Slicer 由管理器传入 `UR12E_CONTROL_RUNTIME_DIR`，无须手工拼接个人路径。保持这些 cache 位于 Linux 文件系统，mock 默认控制路径不变。

安装日志保存在入口打印的 `output/ursim_install/<UTC时间>.log`。网络、TLS、摘要或包安装失败时保留日志与失败记录，解决原因后重跑；不禁用 TLS，不引入来源不明的证书、镜像或 URCap。安装成功只证明资源准备，不证明控制程序已运行。

## 3. 首次虚拟示教器配置

先停止本项目正在运行的 mock，使用它自己的 `stop_linux.sh`；不要同时启动两个后端。然后只启动虚拟控制柜：

```bash
bash ur12e_slicer/start_ursim_linux.sh --simulator-only
bash ur12e_slicer/status_ursim_linux.sh
```

`--simulator-only` 启动已核验的容器和本机 GUI relay，不启动 ROS / Slicer、不发送运动。容器 / 网络名带项目哈希，管理器拒绝其他镜像、型号、远程 Docker、额外网络成员和外部机器人 IP。

在 Docker 所在主机浏览器打开 [本机虚拟示教器](http://127.0.0.1:6080/vnc.html)。noVNC 设置中可选本地缩放（Local Scaling），使完整示教器适应窗口。按以下顺序完成首次配置：

1. 确认这是本任务的 UR12 虚拟控制柜；完成默认仿真安装的首次确认。保留由 UR12 生成的默认安装，不复制旧 UR5 示例安装文件。
2. 在 `Installation → URCaps → External Control` 中，将主机 IP 设为 **`192.168.62.1`**，端口设为 **`50002`**。这指向本机 ROS 驱动，不是控制柜自身的 `.2`。
3. 用 `Save Installation As` 保存为 **`default`**。预置 `external_control.urp` 引用该安装；不只改输入框而漏掉保存。已有 External Control 1.0.5 由挂载安装，应核对界面显示。
4. 在 `Settings → System → Remote Control` 启用远程控制，然后回主界面右上角将当前控制模式切到 **Remote Control**。只启用功能而仍处于 Local 模式不足以让启动器控制程序。
5. 回到终端继续全栈启动。启动器负责本机 Dashboard 的上电、制动释放、加载并播放 External Control 程序；不要同时手工播放其他程序。

Docker 29 的内部桥上，GUI `-p` 发布在本机验证中不可用；本任务使用 Python relay 绑定 `127.0.0.1:6080`，仅转发到已核验的本机容器。不要为打开 GUI 改成 host 网络或开放实验室网段。端口已被其他进程占用时先核对占用者，不能停止其他任务来抢端口。

## 4. 启动 ROS / MoveIt 和 Slicer

完成首次配置后，在仓库根目录执行：

```bash
bash ur12e_slicer/start_ursim_linux.sh
bash ur12e_slicer/status_ursim_linux.sh
```

此时复用本任务容器，启动独立 `ursim.launch.py` 与 Linux Slicer。ROS launch 固定 `ur_type=ur12e`、`use_mock_hardware=false`、虚拟控制柜 `.2` 和回连 `.1`，激活 `scaled_joint_trajectory_controller`；桥也明确选择该控制器。`false` 在此处表示使用 URSim 通信接口，不提供真实机器人 IP 切换参数。

只启动 ROS 而暂不开 Slicer 时使用 `start_ursim_linux.sh --backend-only`。该模式不证明 GUI 已验证。启动结果应同时满足：容器身份正确；External Control 程序处于运行；ROS controller / 规划服务 / action 可用；status 输出的 `control_runtime` 下 `state.json` 为 `source=ros2_ursim`、`ready=true`、`feedback_valid=true`，关节和 TF 反馈持续新鲜；Slicer 采用当前来源标签。不要读取项目内旧 `runtime/state.json` 代替 URSim 控制文件。单独的容器启动或 `ready=true` 均不能证明运动执行。

Slicer 复用现有针尖目标、规划预览、缓存执行和反馈界面。针尖仍为 `tool0` 局部 +Z 120 mm，协议仍为 `base` 下米和 `xyzw` 四元数；当前没有真实工具 / 影像 / 光学标定。目标从实际针尖附近设置，由本轮 MoveIt 规划判断可达性。

## 5. 基础控制验收与停止

先暂停 Slicer 手工发目标；同一文件桥只允许一个控制端。运行：

```bash
bash ur12e_slicer/run_ursim_linux.sh
```

基础验收包含控制柜 / ROS 描述身份检查、弯肘初位、第一关节小位移与返回、针尖附近目标的实际 MoveIt 规划和缓存执行，以及新鲜 action / joint / 文件桥结果核对。不带 `--no-slicer` 时，随后在本轮 Slicer 内运行实际目标 / 规划 / 执行 / 反馈验收，只有新产生的完整结果通过才算全栈通过。它会发送仿真运动命令。离线单元检查、API 存在、前端动画或旧结果不能替代本轮返回。

- 仅后台运行用 `run_ursim_linux.sh --no-slicer`，不验证 Slicer GUI。
- 结束后清理本任务组件用 `run_ursim_linux.sh --stop-after`；可与 `--no-slicer` 合用。
- 不带 `--stop-after` 时，完成查看后运行 `bash ur12e_slicer/stop_ursim_linux.sh`，再查 `status_ursim_linux.sh`。

本轮外层结果位于 `output/ursim_validation/<UTC标识>/lifecycle_summary.json`，最新指针为 `output/ursim_validation/latest.json`。基础原始记录先写原生 Linux cache 的 `ursim/validation/`，运行成功或失败后都以 `copyfile` 逐字节复制到上述项目输出目录，逐文件核对 SHA256；`archive_provenance.json` 和外层汇总保留原始 cache 位置。归档只复制字节，不要求跨文件系统复制元数据，也不改实验数值。

外层引用的子目录 `summary.json` 记录每项基础动作、目标 / 实际关节、误差、MoveIt / controller 返回、事件及清理失败。先确认两个汇总 `passed=true`，再查看 `no_slicer`、`stop_passed` 等实际执行范围。全栈还应有 `slicer_passed=true`，`slicer_result` 指向本轮 `output/ursim_slicer_acceptance/run_<标识>/results.json`，其中 `all_passed=true`、来源 `ros2_ursim`。原始日志与失败轮次保留。Slicer 面板验收与 ROS 基础动作分别保存，不能仅因启动了 GUI 而称面板操作验收完成。

停止入口核对 PID、进程启动信息、容器 ID、网络和 Docker socket 后只处理本任务；身份不符或残留会报错并保留记录。停止后不删除安装、程序与日志。本次记录包含 Docker `137`、MoveIt `-11`、ros2_control `-15` 等退出异常；它们与验收停止时无活动残留分别保留，不能由“无活动进程”推断所有组件无错误退出，也不构成硬件安全停止性能验证。

## 6. Windows + WSL 本机启动保持

原生 Linux 直接使用上面的 shell 入口。Windows 本机使用已安装的 WSL 环境时，后台 systemd 服务本身不能保证空闲 WSL 会话持续存活；本项目提供 `URSim_Windows.ps1` 和 `bootstrap/ursim_wsl_session.sh` 保持启动会话。

在仓库根目录的 PowerShell 中明确传入本机已存在的发行版名、普通用户：

```powershell
& '.\ur12e_slicer\URSim_Windows.ps1' -Action start -Distribution '<WSL发行版名>' -LinuxUser '<普通用户>'
& '.\ur12e_slicer\URSim_Windows.ps1' -Action status -Distribution '<WSL发行版名>' -LinuxUser '<普通用户>'
```

也可在本机 `ur12e_slicer/runtime/ursim_windows.json` 保存：

```json
{"distribution": "<WSL发行版名>", "user": "<普通用户>"}
```

该配置不提交，不在公开说明中固定个人用户名或绝对路径。配置后可用 `Start_URSim_Demo.cmd` / `Stop_URSim_Demo.cmd`；启动器隐藏保持窗口，日志位于 `output/ursim_windows/`。启动派发消息只表示进程已派发，随后必须运行 status 核对实际 ROS / Slicer 就绪。包装器 `-Action run` 执行基础验收，`-Action stop` 关闭本任务。首次 simulator-only 的 GUI 配置仍按第 3 节在选定 WSL 普通用户会话内完成，并保持该会话打开。

## 7. 本机 WSL 时钟修正记录

本节仅记录 2026-10-27 本机专用 **`UR12e-Jazzy`** WSL2 发行版的调试变更，不是原生 Ubuntu 的通用安装步骤，URSim 安装入口也不自动修改这些服务。

本机采样发现 wall clock 约每 31 秒后退 0.832 秒，影响 ROS / TF 时间戳与文件状态新鲜度。排查到宿主 Hyper-V 同步与来宾 `systemd-timesyncd` 并发校时后，在该专用发行版停止并禁用来宾服务：

```bash
sudo systemctl stop systemd-timesyncd
sudo systemctl disable systemd-timesyncd
```

宿主 Hyper-V 时间同步继续。随后约 42.017 秒采样未见回退；本机日志为 `output/ursim_setup_2026-10-27/wsl_clock_adjustment.log`。没有扩大反馈年龄、未来时间或帧间偏差门槛来掩盖问题。将来恢复来宾服务时，在该发行版运行：

```bash
sudo systemctl enable --now systemd-timesyncd
```

恢复后重新采样并检查实际反馈新鲜度。此记录不表示已证明长期时钟稳定性、硬件时延或 WSL 实时控制性能。

## 8. 当前证明边界与真机接续

URSim 提供控制柜协议和程序行为检查，不提供真实机械臂动力学、接触、碰撞或安全停止性能。官方列有模拟器限制，见 [Linux URSim 说明](https://www.universal-robots.com/download/software-ur-series/simulator-linux/offline-simulator-ur-series-e-series-ur-sim-for-linux-5261/)；容器准备另见 [官方 Docker 使用说明](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/jazzy/doc/ur_client_library/doc/setup/ursim_docker.html)。当前头部影像、光学观测与独立目标仍是公共示例 / 合成几何；MR 表面未经解剖审核，也没有加入 MoveIt 头模碰撞。

后续真机接续还需确认实际控制柜软件、本体校准、真实工具 TCP / 负载与安装、网络与共享机时；光学参与时另需定位仪、参考架、工具和影像标定及实体独立计量。当前没有真机、光学硬件联调、临床或 ASTM 合规结果。本次 WSL2 / WSLg 结果也不等于另一台全新原生 Linux 主机从零安装已通过。
