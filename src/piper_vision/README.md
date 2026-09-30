# piper_vision 使用说明

该包使用 Ultralytics YOLO11 Detect 模型识别 RGB 图像中的目标，并结合
Orbbec 对齐深度、相机内参和手眼 TF，将目标中心转换到机械臂
`base_link` 坐标系。

## 1. 环境与模型

YOLO 节点默认通过名为 `yolo11` 的 Conda 环境运行。该环境需要能够同时
导入以下模块：

```bash
conda run -n yolo11 python -c \
  "import ultralytics, rclpy, cv_bridge; print(ultralytics.__version__)"
```

模型必须是 YOLO Detect 类型的 `.pt` 权重。启动时使用绝对路径传入，模型
不需要复制到 ROS 包中。对于电梯按钮应用，模型的类别列表必须实际包含
对应按钮类别。

## 2. 启动

先启动 Orbbec，并启用深度到彩色图的配准：

```bash
ros2 launch orbbec_camera dabai_dcw2.launch.py depth_registration:=true
```

机械臂驱动和 `robot_state_publisher` 还需要发布
`base_link -> ... -> link6`。随后启动手眼 TF 和 YOLO11：

```bash
ros2 launch piper_vision yolo_handeye.launch.py \
  model_path:=/path/to/best.pt \
  conda_env:=yolo11 \
  interest:=all
```

常用参数：

- `model_path`：YOLO11 Detect 权重的绝对路径，必填。
- `conda_env`：运行 YOLO 节点的 Conda 环境，默认 `yolo11`。
- `device`：Ultralytics 计算设备；留空自动选择，也可设为 `cpu` 或
  `cuda:0`。
- `interest`：要发布到目标话题的模型类别名；`all` 表示全部类别。
- `conf_threshold`：检测置信度阈值，默认 `0.7`。
- `iou_threshold`：非极大值抑制 IoU 阈值，默认 `0.45`。
- `depth_threshold`：最大有效深度，单位米，默认 `2.0`。
- `depth_scale`：原始深度单位到米的比例，Orbbec 毫米深度默认
  `0.001`。
- `box_roi_inset`：从检测框四边向内缩进的比例，默认 `0.25`。节点对
  中心区域有效深度取中位数。
- `target_frame_id`：三维输出坐标系，默认 `base_link`。
- `camera_frame_id`：留空时使用 CameraInfo 的 optical frame，通常是
  `camera_color_optical_frame`。
- `bg_removal`：是否在推理前用灰色替换无效或过远背景。
- `plane_outer_scale` / `plane_inner_scale`：按钮框外围平面拟合环的外、
  内尺度，默认 `2.0` / `1.0`。
- `plane_ransac_threshold`：RANSAC 平面内点距离阈值，默认 `0.003 m`。
- `plane_min_points` / `plane_min_inlier_ratio`：最少平面点数与最低内点
  比例，默认 `100` / `0.6`。
- `plane_max_rms`：拟合平面的最大均方根误差，默认 `0.004 m`。
- `plane_max_depth_deviation`：外围点相对中位深度的最大偏差，默认
  `0.03 m`。
- `plane_lock_sample_count`：任务开始时锁定面板所需的连续合格平面数，
  默认 `5`。
- `plane_lock_max_offset_spread` / `plane_lock_max_angle_spread`：锁定前
  允许的平面法向距离波动和法向角度波动，默认 `0.005 m` / `3°`。

运行期间可修改感兴趣类别：

```bash
ros2 service call /set_interest piper_msgs/srv/SetInterest \
  "{name: 'button'}"
```

每次调用 `/set_interest` 都会清除上一任务的面板锁定。节点先在可靠深度
视角累计稳定面板；锁定后按钮位置由当前 RGB 中心射线与该面板求交得到，
不再使用可能失真的近距离按钮中心深度。

## 3. 输出话题

- `/piper_vision/all_object_points`：全部有效检测的三维位置和尺寸。
- `/piper_vision/target_point`：与 `interest` 匹配的目标；当其为
  `all` 时发布所有目标。
- `/piper_vision/button_pose`：选定按钮的完整位姿。位置为按钮中心，
  姿态的 `+Z` 轴为锁定的按压方向；只有 `interest` 是模型中一个明确
  类别且任务面板已锁定、当前 RGB 射线能安全与面板求交时才发布。
- `/piper_vision/pred_image`：带 YOLO11 检测框的 RGB 图像。

前两个话题的 `header.frame_id` 都是 `target_frame_id`。三维点先在
`camera_color_optical_frame` 中由 RGB-D 数据生成，再通过手眼 TF 转换。
当彩色/深度图尺寸不一致、检测框中没有有效深度或图像时间戳对应的 TF
不可用时，该帧目标不会发布。

查看指定按键在 `base_link` 下的实时位姿：

```bash
ros2 run piper_vision button_pose_viewer key_3
```

也可以使用 ROS 参数：

```bash
ros2 run piper_vision button_pose_viewer --ros-args \
  -p button_name:=key_3
```

查看器会调用 `/set_interest` 选择目标类别，并打印位置、四元数和 RPY。
超过 `pose_timeout`（默认 `1.0 s`）没有收到该按键的有效位姿时，所有
位姿数值显示为 `0`。输出周期默认 `0.5 s`，可通过 `output_period` 参数
修改。

手眼标定矩阵的语义是 `link6 → camera_link`，因为 easy_handeye 标定时
使用的 tracking base frame 是 `camera_link`。Orbbec 自身发布
`camera_link → camera_color_optical_frame`；视觉节点在 optical frame
反投影深度点，再经完整 TF 链转换到 `base_link`。不要把同一矩阵直接
标成 `link6 → camera_color_optical_frame`，也不要同时运行两套手眼 TF
发布器。

## 4. 语义地图构建

启动 YOLO11 节点后运行：

```bash
ros2 run piper_vision vlm_mapper_node
```

触发建图，结果保存在当前目录的 `map/map.json`：

```bash
ros2 service call /piper_vision/map_capture std_srvs/srv/Empty "{}"
```

`piper_vision_api.py` 中的 `get_coordinate_by_name` 可用于查询坐标。
