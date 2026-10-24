# Linux 部署、运行与验收

更新日期：2026-10-24。目标环境为 **Ubuntu 24.04（Noble）、x86_64 / amd64 桌面系统**。本说明服务于接收仓库的操作者与 Codex。源码不依赖原作者的 Windows 路径或 Codex 私有运行时。

当前交付验证的是 UR12e mock、真实 ROS / MoveIt 接口、Slicer 显示和合成导航误差评价。尚未接物理光学定位仪或真实机械臂；不能用结果表示实物精度、临床效果或 ASTM 标准符合性。

## 1. 环境与依赖

| 项目 | 要求 / 用途 |
|---|---|
| 系统 | Ubuntu 24.04、amd64；完整 GUI 演示需要桌面、可用显示环境与 OpenGL |
| 权限 | 安装系统依赖时可用 `sudo`；克隆、构建、启动和实验用普通用户 |
| 网络 | 可连接 Ubuntu / ROS 官方 apt 源、GitHub、Slicer 官方资源入口；下载失败保留日志并停止 |
| Python | Ubuntu 系统 Python 3.12、`python3-numpy`、`python3-yaml`；不使用旧 Anaconda |
| ROS | ROS 2 Jazzy，`ros-jazzy-desktop`、`ros-dev-tools`、MoveIt 2、UR 官方包、控制器和 TF 工具 |
| Slicer | Linux amd64 版 5.12.4，使用其自带 Python；不向系统 Python `pip install slicer` |
| 图形与中文 | Slicer 所需 Qt / XCB / OpenGL 系统库；`fonts-noto-cjk` 用于本项目中文标签，由安装入口准备 |
| 构建 | colcon；安装步骤提供依赖；工作区必须由当前用户可写 |

不需要 Windows、WSL、Docker、Gazebo、URSim 或 Node.js。没有图形环境时可先做离线评价和 mock 后台检查；这不等于完成 Slicer GUI 验收。其他 Linux 版本、arm64 和无显示服务器不在这次完整交付支持目标中。

原作者的 Windows + WSL 参考运行曾使用 Ubuntu 24.04.5、ROS 2 Jazzy、MoveIt 2 2.12.4、UR 驱动 3.9.0、Slicer 5.12.4。ROS apt 仓库会更新；安装步骤记录本机实际包版本，并检查安装版是否支持 `ur12e`。不要将版本相近的 UR10e 改名使用。

## 2. 获取仓库

在当前用户有写权限的位置克隆：

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 https://github.com/houqingyang763-eng/3d-slicer.git
cd 3d-slicer
git rev-parse HEAD
```

预期：命令退出 0，最后一行是本机取得的提交 ID。保存它作为复现实验的源码版本。

当前文件树已移除早期 Windows 软件与工作缓存，旧 Git 历史仍含这些大文件；早期 CT 练习场景保留少量 LFS 指针。`--depth 1` 省去旧提交；`GIT_LFS_SKIP_SMUDGE=1` 只跳过当前 Linux 任务不需要的旧 LFS 实体。不要执行全量 `git lfs pull`。当前 Linux Slicer、模型和必需数据按后续安装步骤恢复。新 Windows 环境需要另行准备软件，Linux 不运行旧 `.exe` / `.dll`。

已有克隆先 `git status`，保留自己的修改后正常 `git pull --ff-only`。不要用 `git reset --hard` 清理别人的实验。

## 3. 安装与资源准备

```bash
bash ur12e_slicer/bootstrap/install_linux.sh
```

建议使用普通用户执行，由脚本在安装系统包时请求 `sudo`；安装器也允许由 root 安装系统依赖，但不会启动应用。构建、启动、评价与 Slicer 均使用普通用户。不要在 `sudo bash` 中启动 Slicer。预期：安装退出 0，软件 / 资源检查成功，控制台给出日志与实际版本记录。安装成功只证明依赖就绪，仍需后面的运行检查。

若已经手工安装 Linux Slicer，可在执行安装、启动和评价入口前指定真实可执行文件：

```bash
export SLICER_EXECUTABLE="$HOME/Applications/Slicer-5.12.4-linux-amd64/Slicer"
test -x "$SLICER_EXECUTABLE"
```

环境变量应指向 `Slicer` 启动文件，不能指向安装目录、Windows `Slicer.exe` 或系统 Python。不要只移动安装目录中的单个二进制文件。安装目录应完整保留。未覆盖时使用安装步骤准备的 `ur12e_slicer/.local/slicer/Slicer`。自定义路径由操作者管理，依赖检查不会把它当成已核验版本；应另外核对界面版本及来源。

依赖只读检查，不安装、不联网、不请求 sudo、不启动应用：

```bash
bash ur12e_slicer/bootstrap/install_linux.sh --check-only
python3 ur12e_slicer/bootstrap/check_linux.py --json
```

预期：退出 0，JSON 的 `passed=true`；版本与已验证参考不一致时保留提示并继续做实际运行检查。已缓存资源且系统依赖已经安装时，可用 `install_linux.sh --offline` 离线恢复 / 检查，不能用它代替首次系统依赖安装。`--skip-slicer` 用于只准备离线 / ROS 模式，不代表 GUI 条件已满足。

构建目录可覆盖为当前用户可写位置，例如：

```bash
export UR12E_BUILD_DIR="$HOME/.cache/ur12e_slicer_ws"
```

未覆盖时默认目录为 `${XDG_CACHE_HOME:-$HOME/.cache}/ur12e_slicer/<项目路径哈希>/ros2_ws`。使用独立、没有空格的 colcon 构建目录；源码目录可在含空格的路径，但所有命令都要引用路径。不要复用另一 ROS 项目的 `build` / `install`。首次构建和修改 ROS 包后会由启动入口处理，不把原作者的编译结果拷到新机器。

安装失败时不继续启动。先查看输出的安装日志、退出码、版本清单和失败下载地址。确认官方源 / 代理、系统版本和权限后重跑同一入口，不跳过哈希或依赖检查。

## 4. 先验证离线软件

这些命令不发送机器人运动请求，也不需要打开 Slicer：

```bash
python3 -m unittest discover -s ur12e_slicer/navigation_benchmark -p 'test_*.py' -v
python3 ur12e_slicer/validation/check_navigation_metrics.py
PYTHONPATH=ur12e_slicer/ros2_ws/src/ur12e_slicer_bridge \
  python3 -m unittest discover \
  -s ur12e_slicer/ros2_ws/src/ur12e_slicer_bridge/tests -v

python3 ur12e_slicer/navigation_benchmark/run_offline.py \
  --output "$PWD/ur12e_slicer/output/my_offline_run"
```

预期：单元测试与指标检查退出 0；`my_offline_run/` 下有 `truth.json`、`fixture.json`、`run.json`、`cases.csv`、`report.html`。`run.json` 应记录 12 类 × 6 个目标共 72 项、42 个有效与 30 个预期拒绝，解析 / 真值不变性检查通过。拒绝案例的误差为空，不是零误差。

用浏览器直接打开 `report.html`。这些结果来自固定人为扰动，只说明程序能识别已知几何和失效问题，不说明本机硬件定位性能。输出到新目录以保留每次结果。固定输入复跑可加 `--fixture <已有fixture.json>`，详细 API 见 [离线程序说明](navigation_benchmark/README.md)。

## 5. 启动 mock 与 Slicer

```bash
bash ur12e_slicer/start_linux.sh
bash ur12e_slicer/status_linux.sh
```

预期：本任务 ROS 后台就绪，`runtime/state.json` 中的 `source` 为 `ros2_mock_hardware`、`ready=true`、`feedback_valid=true`，反馈持续更新；Slicer 打开本任务面板。启动完成前不可依靠旧的 `state.json` 判断成功。

`status_linux.sh` 输出 backend / slicer JSON 状态，只读，不启动或停止应用。暂不打开 GUI 时用 `start_linux.sh --backend-only`，后续 `run_navigation_linux.sh --no-slicer` 检查 ROS mock；该模式明确跳过 Slicer 场景和重载。

ROS 使用 `ROS_DOMAIN_ID=62`、`ROS_LOCALHOST_ONLY=1`，仅本机 mock；普通 `joint_trajectory_controller` 执行 MoveIt 轨迹。mock 不发布 Gazebo `/clock`，使用系统时间。不要将 `use_sim_time` 改成 `true`。

Slicer 基础面板支持设置目标、规划、执行缓存计划和查看反馈。目标不是预先保证可达；先用当前针尖附近目标，再由真实 MoveIt 规划判断。通用面板基础场景与头部导航评价场景是两个使用入口，头部链由下一节评价程序构造。

## 6. 跑完整导航评价

```bash
bash ur12e_slicer/run_navigation_linux.sh
```

执行期间不要在 Slicer 另发目标，也不要同时启动第二个评价器。该过程运行固定离线场景，以及基于影像 / 模拟光学坐标链的真实 MoveIt 规划、mock 执行与反馈检查，并生成本轮结果。

预期：脚本退出 0，打印本轮输出路径。汇总在 `output/linux_validation/<UTC标识>/summary.json`，最新汇总 `output/linux_validation/latest.json`；`passed=true` 仅针对文件中列出的执行模式。离线报告位于 `output/navigation_offline/<本轮标识>/report.html`；联动最新结果为 `output/navigation_benchmark/latest_result.json`，详细 `run.json` 有 3 个执行场景与 6 个执行前拒绝场景，`cases.csv`、`report.html` 与完整输入、请求和实际末态同时保留。

不带跳过参数的完整运行还会在 Slicer 生成头部场景、MRB 和截图，并启动独立 Slicer 进程重新加载场景。检查汇总中的 `slicer_scene` / `scene_reload` 与相关日志。`--offline-only` 只做纯 Python 检查；`--no-slicer` 包含真实 ROS mock，但不检验 GUI；`--stop-after` 运行结束后调用本任务停止流程。未加 `--stop-after` 时，查看完结果按下一节主动停止。

查看结果时重点区分两种误差：

- `robot_command_residual`：mock 工具实际反馈到“估计命令点”的残差，工程检查采用 ≤2 mm / 整体角 ≤0.03 rad。
- `true_target_error`：针尖到独立保存的合成影像目标的误差；含预置标定偏差的场景会超过上述命令残差预算，这正是该检查要保留的现象。

不要要求每个最终目标误差都小于 2 mm，也不要要求不同 MoveIt 运行的末位数值完全一致。六类拒绝包括工具遮挡、过期帧、整包重放、头 / 工具时间不齐、规划后头部移动及标定改变；应无新执行命令、无关节运动。报告内模拟真值、光学数据与 MR 合成点不可当作外部物理测量。

## 7. 正常停止

```bash
bash ur12e_slicer/stop_linux.sh
```

预期：入口只停止本任务已记录并校验身份的前端 / mock 后台，等待进程退出；退出 0 后不应仍有该任务的活动进程。保存停止日志。若进程身份不符、超时或仍有残留，停止命令应报错，先核对 PID / 启动记录，不使用 `pkill ros2`、`killall` 清理所有 ROS / Slicer。

历史 Windows + WSL 停止检查观察到 MoveIt / 控制器退出错误，即使任务进程组最终为空也保留记录。本机是否出现同类问题应以本次日志为准，不能由“无残留”推断所有组件正常退出。

## 8. 结果、参考证据与提交范围

| 类型 | 解释 |
|---|---|
| [参考结果与来源](reference_results/2026-10-23/PROVENANCE.md)、[离线报告](reference_results/2026-10-23/offline/report.html)、[MoveIt 报告](reference_results/2026-10-23/moveit_mock/report.html) | 原作者历史参考运行，执行环境为 Windows + WSL / Slicer；可用于理解报告，不是接收方本机运行证据 |
| 安装 / 启动入口本次打印的日志和新输出目录 | 当前机器的版本、时间、退出码和实跑证据；验收以这些文件为准 |
| `runtime/` 中的状态、锁与 PID | 本机临时文件，不能复制到另一台机器复用 |
| 构建目录、Slicer 安装、模型下载缓存 | 按来源与版本恢复的外部依赖，不作为源码提交 |

### 本次部署验证的环境与复用范围

2026-10-24 的迁移检查在 **Windows 宿主的 WSL 2、Ubuntu 24.04.5 x86_64、WSLg** 中开展，使用普通用户 `ur12echeck` 和独立 HOME 下的 `release-check` 源码目录。Slicer 是真正的 **Linux 5.12.4 二进制**，不是通过 Windows Slicer 代替。工作副本没有携带原项目的运行时队列、已恢复模型或编译产物。

- UR 官方描述 ZIP 从空资源目录在线获取，校验后恢复模型。
- Slicer 官方约 498 MB 安装包核对 SHA512 与 SHA256 后，在 Linux ext4 的新目录解压恢复；复用已核验的下载缓存，避免重新下载相同大文件。
- 复用该 Ubuntu 已安装的 ROS apt 软件环境，重新为普通用户构建项目包。

因此，这次检查覆盖独立源码目录、普通用户权限、资源恢复、Linux 入口、实际 ROS mock 和 Linux Slicer。它没有覆盖另一台物理 Linux 电脑，也没有在全新操作系统中从零安装全部 ROS apt 包；接收方仍需按步骤完成并保存本机安装与运行日志。

只读依赖检查与不含 Slicer 的导航运行已通过。补齐中文字体和生命周期修正后的完整运行标识为 **`20261024T090044Z_4e8a81`**，执行 `run_navigation_linux.sh --stop-after` 返回成功，检查结果如下。GUI 截图也已人工检查，中文显示正常。公开证据见 [本次来源与范围](reference_results/2026-10-24-linux/PROVENANCE.md)、[环境记录](reference_results/2026-10-24-linux/deployment_context.json)和 [逐项汇总](reference_results/2026-10-24-linux/validation/summary.json)。

| 检查 | 本次结果 |
|---|---|
| 几何 / 坐标单元测试 | 26 项通过 |
| 文件桥协议与原子写入 | 10 项通过 |
| 资源恢复检查 | 4 项通过 |
| 指标与执行授权 | 10 项通过 |
| 固定离线场景 | 12 类 × 6 个目标＝72 项；有效 / 预期拒绝与解析检查通过，见 [报告](reference_results/2026-10-24-linux/offline/report.html) |
| 实际 ROS / MoveIt mock | 3 次规划执行成功；6 类执行前失效均拒绝，见 [报告](reference_results/2026-10-24-linux/moveit_mock/report.html) |
| Linux Slicer | 真实 Linux 应用生成 MR 头部场景、三视图和结果显示，中文正常 |
| 独立 MRB 重开 | 另起 Slicer 进程重新加载，6 个合成拟合点 / 5 个留出检查点保留，见 [重载记录](reference_results/2026-10-24-linux/checks/scene_reload_verification.json) |
| 停止清理 | 本任务前端与后台进程组无活动成员残留 |

这些通过项对应上述环境与运行标识，不能扩展为尚未执行的实物测试。之后另行完成以下补充检查，没有把不同命令合写为同一次完整运行：

| 补充检查 | 实际范围与结果 |
|---|---|
| 默认安装入口幂等 | `install_linux.sh` 在已有 ROS apt 环境以 root 执行成功；只安装 / 检查依赖，未启应用；不代表全新系统首装，见 [安装日志](reference_results/2026-10-24-linux/supplemental_checks/install_online.log) |
| 更新后的离线回归 | 普通用户运行 `run_navigation_linux.sh --offline-only`，标识 `20261024T090333Z_940920`；资源 4 项、生命周期 12 项及其余离线检查通过，见 [汇总](reference_results/2026-10-24-linux/supplemental_checks/offline_only/summary.json) |
| 真实启动 / 复用 / 停止 | 连续两次 start 的任务 PID 保持不变，复用既有实例；随后 stop 成功，status 中前端 / 后台均不再运行，见 [补充检查](reference_results/2026-10-24-linux/supplemental_checks/summary.json)、[停止状态](reference_results/2026-10-24-linux/supplemental_checks/stopped_status.json) |

仓库另有 [GitHub Actions 离线回归配置](../.github/workflows/linux-software-checks.yml)，只运行离线软件检查，不验证 Linux GUI、真实 ROS 运动或物理设备。线上 Actions 是否通过，以推送后的实际工作流记录为准。

本次也观察到 `move_group` 停止阶段退出码 -11，但任务进程组清理完成。该异常保留为已知限制；“无任务进程残留”只说明清理完成，不表示每个组件均无错误退出。它不构成真实设备停止性能或硬件安全停机验证。

检查公开交付内容可运行 `python3 tools/prepare_ur12e_release.py`；需要独立源码副本时加 `--copy-to <新的空目录>`。它按 `publish_manifest.json` 的显式白名单复制并记录库存，不复制软件、模型安装、runtime、运行 output 或内部周报材料，也不执行 Git 操作。

## 9. 常见问题

| 症状 | 检查与处理 |
|---|---|
| `ROS 2 Jazzy is missing` / ROS 包找不到 | 确认 Ubuntu 24.04；重新运行安装入口并检查安装日志，不能只设置环境变量伪装安装 |
| UR 包没有 `ur12e` 参数 | 查看已安装 launch 和包版本；获取支持 UR12e 的官方 Jazzy 包，不用 UR10e 代替 |
| `numpy` / `yaml` 导入失败 | 确认 `command -v python3` 使用系统 Python；退出抢占 PATH 的 Anaconda / venv，检查 apt 依赖 |
| `Permission denied` / colcon 写 `/root` | 不以 root 运行；设 `UR12E_BUILD_DIR` 为当前用户可写独立目录，检查仓库和 runtime 归属 |
| Slicer 找不到 / 不能执行 | 核对 `SLICER_EXECUTABLE`、Linux amd64 平台和执行权限；使用完整解压目录 |
| Slicer Qt / XCB / OpenGL 报错 | 查看 Slicer 启动日志、桌面显示变量、系统图形依赖；无 GUI 时先做离线 / 后台验证，保留 GUI 未通过状态 |
| 中文显示为方框 | 检查 `fonts-noto-cjk` 是否安装，重开 Slicer 后查看；不要把缺字截图作为正常 GUI 交付 |
| 启动等待、`ready=false` 或反馈不更新 | 查看本轮 ROS 日志、状态中的 `message`；核对 domain 62、JTC、关节 / TF、规划服务与执行 action |
| 旧 PID、锁、已有实例 | 先运行本任务停止入口并核对身份；不要直接覆盖活进程记录，也不要删除其他任务的锁 |
| 时间戳过期 / 未来 | 让评价器与 ROS 在同一 Linux 系统时钟下运行；核对系统时间，不放宽帧龄阈值掩盖问题 |
| 下载超时或校验不符 | 保留失败文件与日志，检查官方来源 / 网络后重试；不要禁用校验或改成未知镜像继续 |
| MoveIt 目标不可达 / 执行失败 | 阅读实际错误码和初态；保持失败结果，不用前端插值替代真实执行 |

更多规则见 [AGENTS.md](../AGENTS.md)。Windows 兼容入口、早期验收及暂停记录保留在 [使用与验收.md](使用与验收.md)、[RESULTS.md](RESULTS.md)、[RESUME.md](RESUME.md)，不作为当前 Linux 安装步骤执行。
