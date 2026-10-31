# URSim 精选证据：10 月 27 日与 10 月 31 日

归档日期 2026-10-31。这些文件来自已经完成的 Windows + WSL2 / WSLg 本机运行，不是发布时重新启动的实验。快速解释见根目录 [PROJECT_CONTEXT.md](../../../PROJECT_CONTEXT.md)。

| 目录 | 原始轮次 | 基础控制 | Slicer / 外层结论 |
|---|---|---|---|
| [2026-10-27-full-pass](2026-10-27-full-pass/lifecycle_summary.json) | `20261027T044855Z_f8a2ba` | 5/5 | 21/21；外层通过，停止检查通过 |
| [2026-10-31-run-1](2026-10-31-run-1/lifecycle_summary.json) | `20261031T084059Z_f5ee4e` | 5/5 | 20/21；只有录帧 19<20 未过，外层失败 |
| [2026-10-31-run-2](2026-10-31-run-2/lifecycle_summary.json) | `20261031T084234Z_fd2c4e` | 5/5 | 20/21；只有录帧 19<20 未过，外层失败 |

每轮的 `basic_control_summary.json` 保存三次关节 action、一次仅规划和一次缓存执行的结果；`slicer_results.json` 保存实际目标、计划、执行、保持反馈、全部 checks 及误差。`lifecycle_summary.json` 保留外层通过 / 失败和 traceback。10 月 31 日没有请求 `--stop-after`，不能把未执行的停止检查记成该轮通过。

## 头部显示

`head-display/` 中两张截图为实际 Slicer 原图。`head_display_summary.json` 记录使用 `ros2_ursim` 反馈、没有发送机器人指令、沿用已有模拟坐标关系。此操作仅加载公共 MR 及其强度表面，不带旧 mock 的执行轨迹、误差成绩或模拟光学评分。

`show_head_display.py` 是当时通过 `slicer/task_client.py` 执行的会话脚本，依赖已存在的 `output/navigation_benchmark/latest_result.json`，没有修改默认启动行为。`measure_head_geometry.py` 与 `head_geometry_measurements.json` 是补充的模型尺寸检查：MR 为 256×256×130 体素，间距约 1×1×1.3 mm，影像到 base 旋转无额外缩放。其线段与等值面交点只是该显示网格的几何交点，不能解释成骨厚、真实入针深度、碰撞安全或临床测量。

## 来源与处理

来源清单为 [evidence_manifest.json](evidence_manifest.json)。每个文件记录本机源位置的仓库相对表示、原字节 SHA256、发布字节 SHA256、字节数和处理规则：

- JSON 仅将本机仓库 / 用户 cache 路径规范化并重新排版；数值、时间、布尔值、检查结果、请求 ID 保留。
- 截图和两个显示 / 几何脚本逐字节复制。未发布含个人录帧路径的执行截图和 GIF，也未修改图片冒充结果。
- 原始 action 事件流、全部 joint 采样、录帧、MRB 和安装包保留在本机，不在此小型证据包中。JSON 内 `output/` 等路径是原运行位置，不表示那些原始文件也已上传。
- `tools/prepare_ur12e_release.py` 会检查本目录清单中的发布 SHA256；旧 mock 归档的真值检查保持原规则。

[发布前离线检查日志](publication/offline_checks.log)记录 bridge 18、lifecycle 26、guardian 6、validator 24，共 74 项通过；没有启停仿真服务。日志中 unsupported-host 用例的 `FAIL: URSim validation requires Linux` 是预期拒绝输出，该用例与测试组均为 OK，不能将这行单独解释为本次测试失败。

所有误差均来自虚拟模型。没有真机 / 物理光学验证、独立实体精度测量、临床效果或 ASTM 合规结论；MR 表面未经解剖分割审核，未加入 MoveIt 头部碰撞模型。停止时无进程残留与全部组件正常退出是不同结论，历史退出异常保留在过程说明中。
