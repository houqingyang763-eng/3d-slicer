# 导航评价仿真结果查看器

`Open_Navigation_Benchmark.cmd` 独立打开 `output/navigation_benchmark/latest_result.json`。也可运行 `Start_Navigation_Benchmark.ps1 -ResultPath <结果文件>`。该入口只开结果查看器，不开文件通信桥或机器人后端，不向机械臂提交目标。真实 Slicer 应用 PID 记录于 `runtime/navigation_benchmark_viewer.json`。

在已打开的 UR12e 仿真面板中可调用：

```python
import sys
sys.path.insert(0, str(PROJECT / "slicer"))
import navigation_benchmark_scene
navigation_benchmark_scene.run()
```

也支持 `navigation_benchmark_scene.build(result, save=True, result_path=None)`。只替换名为 `NAVBENCH_` 的自有节点，保留当前机器人、反馈与控制面板；不修改 `runtime/target.json`。机器人官方网格仍由已有面板的 ROS mock-hardware TF 驱动。独立重载时，仅在结果包含 `robot_feedback_snapshot`、且其 `source` 为 `ros2_mock_hardware` 时，才可依据该真实仿真反馈快照绘制静态机器人。没有反馈快照时不会捏造机器人姿态。

图中公共 MR 头部仅是形态背景；F 为合成配准几何点、C 为合成独立检查点，均不是真实解剖标注。影像 RAS 毫米坐标通过明确的非单位 `T_base_image_mm` 放入机器人 base 毫米坐标。紫色工具与路径是模拟光学观测；青色路径取自记录的 MoveIt 仿真反馈。所谓独立真值仅指单独保留的仿真几何，不是外部计量实测。

保存内容：`latest_scene.mrb`、头部近景 `latest_scene.png`、平台全景 `latest_platform.png`、显示来源与等值面参数 `scene_metadata.json`。MR 表面由阈值、最大连通面和轻度平滑构成，未验证为头皮或颅骨分割；不能用于避障或精度论证。

“MRI 切片”按钮或 `navigation_benchmark_scene.show_slices()` 显示四联视图，并将真实 MR 切片交点移到当前合成目标。切片来自实际加载的公共 MR，不是示意绘图。`set_view(result, "head")` 与 `set_view(result, "platform")` 回到三维近景和全景；视角按场景实际范围及渲染区宽高比取景，避免两侧面板导致裁切。

最低结果结构：`schema_version: 1`、`mode: "simulation"`、`head`、`goal_base_mm`、`actual_tip_base_mm`、`optical_tip_base_mm`。后三者均为 4×4 位姿。`head` 包含 `volume_path`、`T_base_image_mm`、`fiducials_image_mm` 和 `targets_image_mm`；可选 `T_base_camera_mm`、`T_base_reference_mm` 与 `fiducials_observed_base_mm`。`summary` 的键值展示在结果窗，`samples` 中的 `actual_tip_base_mm` 或 `tcp_pose` 绘制已记录反馈路径，`optical_tip_base_mm` 绘制模拟光学路径。

这些是评价程序与坐标链的仿真检查；不代表真实光学定位性能、临床验证或 ASTM 符合性认证。
