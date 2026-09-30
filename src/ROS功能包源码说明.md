# elevate_ws ROS 2 功能包源码说明

## 当前统一运行参数（最新）

以下为当前实际点击配置；粗定位采样时显式覆盖 `distance_mm:=0.0`。
数值采样记录和计算示例中的历史数据仅供追溯，以此处运行参数为准。

```bash
cd /home/xie/wzl/elevate_ws
bash run_wzl.bash \
  auto_enable:=true enable_motion:=true \
  tcp_offset_z:=0.1358 distance_mm:=65.0 \
  coarse_horizontal_offset:=0.026 coarse_vertical_offset:=0.007 \
  coarse_lateral_error_min:=0.021 coarse_lateral_error_max:=0.032 \
  coarse_correction_attempts:=0 home_joint_tolerance:=0.012
```


> 工作区：本仓库根目录（下文命令示例使用 `/home/xie/wzl/elevate_ws`）  
> 文档依据：当前 `src/` 下的实际源码、接口定义、Launch、YAML、URDF/SRDF 和测试  
> 最后整理：2026-08-22  
> ROS 版本背景：代码和原项目说明主要面向 ROS 2 Humble / Ubuntu 22.04  
> 包识别方式：`package.xml`、`colcon list`、构建文件、节点源码、launch、URDF/SRDF、消息接口和配置文件交叉核对

本仓库基于 ROS 2 Humble，集成 Piper 六轴机械臂、Orbbec RGB-D 相机、YOLO 按钮识别、
手眼坐标变换和 MoveIt 规划，用于完成电梯楼层按钮定位、按压、回位及关节归零。
本文侧重说明“每个 ROS 功能包包含什么、彼此如何连接”；实际启动命令见
[运行按电梯任务指令.md](./运行按电梯任务指令.md)，定位参数标定见
[电梯按钮定位调试指南.md](./电梯按钮定位调试指南.md)，电梯业务状态机和算法公式的
展开说明见 [电梯项目流程与算法说明.md](./电梯项目流程与算法说明.md)。

## 1. 阅读范围与结论摘要

当前工作区可被 `colcon` 识别的 ROS 2 功能包共有 **13 个**：

| 序号 | 功能包 | 构建类型 | 主要职责 | 当前定位 |
|---:|---|---|---|---|
| 1 | `orbbec_camera` | `ament_cmake` | Orbbec RGB-D 相机驱动、图像/深度/点云/IMU/TF 发布及设备控制 | 核心硬件驱动 |
| 2 | `orbbec_camera_msgs` | `ament_cmake` + rosidl | Orbbec 驱动专用消息和服务 | 相机接口定义 |
| 3 | `orbbec_description` | `ament_cmake` | 多款 Orbbec 相机的 URDF、网格模型和 RViz 展示 | 相机模型资源 |
| 4 | `piper` | `ament_python` | 通过 CAN 和 Piper SDK 控制单台真实机械臂 | 真机底层驱动 |
| 5 | `piper_description` | `ament_cmake` | Piper 的 URDF/Xacro、Gazebo 和 MuJoCo 模型、网格及 RViz 配置 | 机器人模型资源 |
| 6 | `piper_msgs` | `ament_cmake` + rosidl | Piper 状态、位姿命令、视觉目标和夹爪 Action 接口 | 项目公共接口 |
| 7 | `piper_with_gripper_moveit` | `ament_cmake` | 以 `tcp_link` 为末端、单夹爪控制关节的 MoveIt 配置 | 新的 TCP 抓取配置 |
| 8 | `piper_moveit_config_v5` | `ament_cmake` | 以 `link6` 为机械臂末端、双指关节控制的 MoveIt 配置 | 旧/另一套 MoveIt 配置 |
| 9 | `piper_tf` | `ament_python` | 手眼标定静态 TF 发布及目标点坐标变换 | 坐标系桥接 |
| 10 | `piper_vision` | `ament_python` | YOLO11 RGB-D 三维定位、按钮表面位姿、ChArUco 检测和 VLM 语义地图 | 视觉感知 |
| 11 | `piper_grasp_control_by_ros` | `ament_python` | 接收目标并调用 MoveIt，另提供夹爪 Action Server | 抓取任务控制 |
| 12 | `piper_pbvs_control` | `ament_python` | 单键视觉定位、MoveIt 运动、多位数字逐键回位、确认键及七关节归零 | 当前电梯任务控制 |
| 13 | `piper_launch` | `ament_python` | 安全默认地编排相机、真机、MoveIt、视觉和电梯任务节点 | 当前总启动包 |

`src/piper_sim/piper_gazebo` 和 `src/piper_sim/piper_mujoco` 已从当前源码删除，
因此不再是可构建功能包。`src/piper_control` 也不是 ROS 功能包：
目录内没有 `package.xml`，本文在第 18 节单独说明。

### 1.1 建议阅读顺序

本文首先说明当前电梯任务实际使用的主链路，再逐个解释 13 个 ROS 功能包。
如果只关心当前项目运行，建议按以下顺序阅读：

1. 第 1 节：系统全貌和当前能力边界；
2. 第 6、8、11、12、14、15 节：真机、MoveIt、手眼 TF、视觉、任务控制和总启动；
3. 第 16、17 节：真机下发关系和推荐启动方法；
4. 其余章节：消息包、模型资源、旧 MoveIt 配置、旧抓取原型和辅助能力。

功能包按用途可分为：

| 分层 | 功能包 |
|---|---|
| 设备与模型 | `orbbec_camera`、`orbbec_camera_msgs`、`orbbec_description`、`piper`、`piper_description` |
| 公共接口与坐标 | `piper_msgs`、`piper_tf` |
| 规划与执行 | `piper_with_gripper_moveit`、`piper_moveit_config_v5` |
| 感知与任务 | `piper_vision`、`piper_pbvs_control`、`piper_launch` |
| 旧/实验链路 | `piper_grasp_control_by_ros`、非 ROS 目录 `piper_control` |

### 1.2 系统的主要数据流

```mermaid
flowchart LR
    Camera[Orbbec 相机] --> Driver[orbbec_camera]
    Driver --> RGB[彩色图]
    Driver --> Depth[对齐深度图]
    Driver --> CamTF[相机内部 TF]
    RGB --> Vision[piper_vision / YOLO11 RGB-D]
    Depth --> Vision
    CamTF --> Vision
    RobotTF[piper_description + robot_state_publisher] --> HandEye[piper_tf / handeye_static_tf]
    HandEye --> Vision
    Vision --> Obj[/piper_vision/target_point<br/>ObjectPos]
    Vision --> AllObj[/piper_vision/all_object_points<br/>AllObjectPos]
    Vision --> ButtonPose[/piper_vision/button_pose<br/>PoseStamped]
    SequenceGoal[/run_elevator_sequence<br/>PressButton Action] --> Sequence[elevator_sequence]
    Sequence -->|每位数字 / key_ok| PressGoal
    Sequence -->|每次按键后关节回位| MoveIt
    ZeroService[/return_all_joints_zero<br/>Trigger Service] --> ZeroNode[joint_zero_return]
    ZeroNode -->|arm 后 gripper| MoveIt
    PressGoal[/press_button<br/>PressButton Action] --> PBVS[piper_pbvs_control]
    ButtonPose --> PBVS
    RobotFeedback[/tcp_pose + /arm_status] --> PBVS
    PBVS --> MoveIt[/move_action<br/>粗定位 / 可选推进]
    MoveIt --> Trajectory[FollowJointTrajectory 真机桥]
    Trajectory --> Real[piper 真机 CAN 节点]
    LegacyPoint[/camera_target_point<br/>PointStamped] --> PointTF[piper_tf / tf_transformer]
    PointTF --> BasePoint[/base_target_point<br/>PointStamped]
    BasePoint --> LegacyGrasp[piper_grasp_control_by_ros]
    UserPose[/base_target_pose<br/>PoseStamped] --> LegacyGrasp
    LegacyGrasp --> MoveIt
    MoveIt --> Fake[MoveIt FakeSystem 演示]
    AllObj --> VLM[piper_vision / vlm_mapper_node]
```

当前按钮任务已经形成“任务编排 → 目标选择 → 视觉位姿 → MoveIt 规划/执行 →
真机反馈验收”链路：
Action 目标名经 `/set_interest` 选择唯一 YOLO 类别，视觉节点发布
`base_link` 下的 `/piper_vision/button_pose`，`piper_pbvs_control` 通过 MoveIt
完成粗定位，并可选沿 `base_link X` 或锁定的面板法向推进。独立的
`elevator_sequence` 节点支持一位或两位楼层，把单键任务编排为“逐位数字键 → 每位后
返回初始关节位 → `key_ok` → 返回初始关节位”。例如 10 楼执行
`key_1 → home → key_0 → home → key_ok → home`。当前实现不再发布 `/pos_cmd`，
不执行连续 PBVS 精调；推进只以
目标位姿到达为成功条件，没有力/触觉或按钮灯反馈，不能据此证明按钮实际触发。

旧的通用抓取原型仍是另一条独立链路。它订阅 `/base_target_point`
（`PointStamped`）或 `/base_target_pose`，没有直接订阅 YOLO 的 `ObjectPos`。
因此“YOLO `ObjectPos` → `grasp_server`”仍需要适配，但不影响新的按钮初定位链。

### 1.3 当前控制路径

| 路径 | 控制入口 | 状态来源 | 典型用途 |
|---|---|---|---|
| 真机 CAN | `piper` 订阅关节或末端命令，再调用 `piper_sdk` | Piper CAN 反馈 | 实际机械臂控制 |
| 真机单键任务 | MoveIt 经真机轨迹桥执行粗定位和可选推进 | `/joint_states`、`/tcp_pose`、`/arm_status` 和锁定的按钮位姿 | 数字键或 `key_ok` 定位；默认不推进 |
| 完整电梯序列 | `/run_elevator_sequence` 逐位调用单键 Action，并在每次按键后 MoveIt 回位 | 单键 Action 结果和真实关节反馈 | 一位/两位楼层数字 → 逐键回位 → OK → 回位 |
| MoveIt FakeSystem | MoveIt 内置 `mock_components/GenericSystem` | 仿真 `/joint_states` | 离线规划和 RViz 演示 |

`piper_description` 仍保留 Gazebo Xacro 和 MuJoCo XML 资源，但当前工作区
没有可直接启动它们的 `piper_gazebo` / `piper_mujoco` ROS 功能包。

### 1.4 当前能力边界

- 总入口默认 `auto_enable=false`、`enable_motion=false`、`distance_mm=0`，
  因此默认只做安全规划，不自动使能、不执行真机轨迹、也不向按钮推进。
- 当前没有接触力控制、按钮行程检测、按钮灯识别、电梯楼层反馈或门状态检测。
- 当前不负责呼梯、等待到站、开门判断或移动底盘进出电梯。
- 包名中的 `PBVS` 是历史保留命名；当前主控制算法是稳定视觉目标锁定、MoveIt
  位姿规划和真实反馈验收，而不是逐帧图像误差驱动的连续速度闭环。

## 2. `orbbec_camera`

路径：`src/OrbbecSDK_ROS2/orbbec_camera`

### 2.1 作用

这是 OrbbecSDK 的 ROS 2 C++ 封装，是视觉系统的硬件数据源。核心类
`OBCameraNodeDriver` 负责发现、选择、连接、断线重连和重启设备；
`OBCameraNode` 负责配置传感器流、采集帧、时间戳处理、图像转换、点云生成、
IMU 同步、TF 发布、诊断和控制服务。

包内直接携带 Orbbec SDK 头文件及预编译动态库，覆盖：

- `x64`
- `arm64`
- `arm32`

构建时会根据 `uname -m` 和系统位数选择对应库目录。还可通过 CMake 选项启用
Rockchip 或 NVIDIA Jetson 的 JPEG 硬件解码。

### 2.2 主要可执行程序

| 可执行程序 | 作用 |
|---|---|
| `orbbec_camera_node` | 主相机驱动；注册为 `rclcpp_components` 组件，也可以独立进程运行 |
| `list_devices_node` | 枚举 Orbbec 设备及基本信息 |
| `list_depth_work_mode_node` | 查询相机支持的深度工作模式 |
| `list_camera_profile_mode_node` | 查询相机支持的流配置/分辨率/帧率 |
| `topic_statistics_node` | 对图像话题和 ROS 统计消息做统计 |
| `frame_latency_node` | 测量 Image、PointCloud2、IMU、Metadata、CameraInfo、RGBD、TF 等话题的帧率/延迟 |

主节点既支持普通 `Node`，也支持 `ComposableNodeContainer` 中的进程内通信。

### 2.3 主要输出话题

实际话题以 `camera_name` 形成命名空间，默认 `camera_name=camera`：

| 默认话题 | 类型 | 条件/含义 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 彩色图 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色相机内参与畸变参数 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度图 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度相机参数 |
| `/camera/ir/image_raw` | `sensor_msgs/msg/Image` | 红外图；部分双目型号还会区分左右 IR |
| `/camera/ir/camera_info` | `sensor_msgs/msg/CameraInfo` | 红外相机参数 |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | `enable_point_cloud=true` 时发布纯深度点云 |
| `/camera/depth_registered/points` | `sensor_msgs/msg/PointCloud2` | `enable_colored_point_cloud=true` 时发布彩色点云 |
| `/camera/accel/sample` | `sensor_msgs/msg/Imu` | 独立加速度流 |
| `/camera/gyro/sample` | `sensor_msgs/msg/Imu` | 独立陀螺仪流 |
| `/camera/gyro_accel/sample` | `sensor_msgs/msg/Imu` | 启用同步 IMU 输出时发布 |
| `/camera/accel/imu_info`、`/camera/gyro/imu_info` | `orbbec_camera_msgs/msg/IMUInfo` | IMU 标定参数 |
| `/camera/depth/metadata` 等 | `orbbec_camera_msgs/msg/Metadata` | 帧元数据 JSON |
| `/camera/depth_to_color` 等 | `orbbec_camera_msgs/msg/Extrinsics` | 开启外参发布后输出各流之间外参 |
| `/camera/depth_filter_status` | `std_msgs/msg/String` | 深度滤波状态 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 当前主要报告相机温度 |
| `/tf` 或 `/tf_static` | `tf2_msgs/msg/TFMessage` | `camera_link` 到各传感器 frame/optical frame |

`RGBD.msg` 已在消息包中定义，延迟测试工具也支持该类型，但当前主驱动源码没有直接创建
RGBD 组合消息发布器；主流程仍以独立彩色图、深度图和 CameraInfo 为主。

### 2.4 主要服务

服务位于相机节点命名空间下。实际是否创建与设备能力及对应数据流是否启用有关。

主要服务包括：

- 曝光与增益：`get/set_[color|depth|ir]_exposure`、
  `get/set_[color|depth|ir]_gain`、`set_[color|depth|ir]_auto_exposure`
- 白平衡：`get/set_white_balance`、`get/set_auto_white_balance`
- 流控制：`toggle_color`、`toggle_depth`、`toggle_ir`
- 激光与保护：`set_laser_enable`、`get_laser_status`、
  `set_ldp_enable`、`get_ldp_status`、`get_ldp_protection_status`
- 设备能力：`get_device_info`、`get_sdk_version`
- 保存数据：`save_images`、`save_point_cloud`
- 其他：`set_fan_work_mode`、`set_floor_enable`、`switch_ir`、
  `set_ir_long_exposure`、`get_ldp_measure_distance`
- 驱动层设备重启：`reboot_device`

服务的请求/响应类型来自 `orbbec_camera_msgs` 和 `std_srvs`。

### 2.5 重要参数

参数数量很多，可按功能分组理解：

- 设备选择：`serial_number`、`usb_port`、`device_num`、`vendor_id`、
  `product_id`、`net_device_ip`、`net_device_port`、`enumerate_net_device`
- 彩色/深度/红外流：`enable_*`、`*_width`、`*_height`、`*_fps`、
  `*_format`、`*_qos`、`*_camera_info_qos`
- 图像对齐：`depth_registration`、`align_mode`
- 点云：`enable_point_cloud`、`enable_colored_point_cloud`、
  `cloud_frame_id`、`ordered_pc`
- IMU：`enable_accel`、`enable_gyro`、采样率、量程、
  `enable_sync_output_accel_gyro`
- TF 和时间：`publish_tf`、`tf_publish_rate`、`use_hardware_time`、
  `time_domain`、`enable_sync_host_time`
- 深度算法：`depth_work_mode`、`depth_precision`、`device_preset`、
  `depth_filter_config` 及 decimation/HDR/threshold/spatial/temporal/
  hole-filling 等滤波器开关
- 同步触发：`sync_mode`、各类 delay、`trigger_out_enabled`、
  `enable_frame_sync`
- 设备功能：`enable_laser`、`laser_energy_level`、`enable_ldp`、
  `enable_heartbeat`

对于本项目的 YOLO RGB-D 定位，最关键的是：

```bash
ros2 launch orbbec_camera <对应型号>.launch.py depth_registration:=true
```

因为 `piper_vision/yolo_detect_3d.py` 明确要求彩色图和深度图分辨率一致，否则拒绝处理。

### 2.6 launch 文件

包内约 40 个型号/场景 launch，主要分为：

- Astra/Astra2/Astra Pro 系列
- Dabai/Dabai DCW/DCW2/DW/Max 系列
- Gemini 2、2L、2XL、330 系列
- Femto/Femto Bolt/Femto Mega
- 单网络相机、多网络相机
- 多相机及多相机硬件同步
- 通用 `ob_camera.launch.py`
- 进程内通信示例

它们的主体逻辑相同，差别主要在默认分辨率、格式、帧率、IR 数量、IMU、深度模式、
网络参数和同步参数。

## 3. `orbbec_camera_msgs`

路径：`src/OrbbecSDK_ROS2/orbbec_camera_msgs`

### 3.1 作用

纯 rosidl 接口包，为 `orbbec_camera` 提供 ROS 标准消息无法完整表达的设备信息、
流外参、IMU 标定、元数据及通用参数读写服务。

### 3.2 消息

| 消息 | 字段与用途 |
|---|---|
| `DeviceInfo` | Header、设备名、序列号、固件版本、最低 SDK 版本、硬件版本 |
| `Extrinsics` | Header、3×3 旋转矩阵、3 维平移 |
| `IMUInfo` | 噪声密度、随机游走、参考温度、偏置、重力、比例/非正交矩阵、温漂 |
| `Metadata` | Header + JSON 字符串 |
| `RGBD` | Header、RGB/Depth CameraInfo、RGB 图和深度图 |

### 3.3 服务

| 服务 | 含义 |
|---|---|
| `GetBool` | 返回布尔值、成功标志和消息 |
| `GetInt32` | 返回整数、成功标志和消息 |
| `GetString` | 返回字符串、成功标志和消息 |
| `SetInt32` | 设置整数，返回成功标志和消息 |
| `SetString` | 设置字符串，返回成功标志和消息 |
| `GetCameraInfo` | 返回 `sensor_msgs/CameraInfo` |
| `GetDeviceInfo` | 返回 `DeviceInfo`、成功标志和消息 |

该包自身不运行节点。

## 4. `orbbec_description`

路径：`src/OrbbecSDK_ROS2/orbbec_description`

### 4.1 作用

提供 Orbbec 相机的外观网格、URDF/Xacro、各传感器 frame 与 optical frame 的固定关系，
用于 RViz 展示、仿真建模或把相机挂到机器人模型上。

覆盖的主要型号：

- Astra2
- Femto Bolt
- Gemini2
- Gemini2L
- Gemini 335/336
- Gemini 335L/336L

模型通常包含：

- `camera_link` 或相机 base link
- 安装螺孔 frame
- depth frame / depth optical frame
- color frame / color optical frame
- 左右 IR frame / optical frame
- 部分型号的 IMU frame

Optical frame 按 ROS 光学坐标约定设置旋转。模型中的标称外参适合展示和初始建模，
真实运行时优先使用相机驱动发布的标定外参。

### 4.2 launch

`view_model.launch.py`：

1. 接收 `model:=<urdf/xacro 文件名>`；
2. 把 Xacro 转成临时 URDF；
3. 启动 `robot_state_publisher`；
4. 启动 RViz 并加载 `rviz/urdf.rviz`。

### 4.3 当前声明问题

`package.xml` 只声明了 `ament_cmake` 和测试依赖，没有显式声明运行 launch 所需的
`xacro`、`robot_state_publisher`、`rviz2`、`launch_ros` 和
`ament_index_python`。环境已安装这些依赖时仍可运行，但新机器上仅靠 rosdep
不一定能完整安装。

## 5. `piper_msgs`

路径：`src/piper_msgs`

### 5.1 作用

Piper 项目各包共用的接口定义，包括机械臂状态、笛卡尔位姿命令、视觉目标集合和夹爪
Action，以及按钮任务 Action。

### 5.2 当前由 CMake 实际生成的接口

| 接口 | 用途 |
|---|---|
| `PiperStatusMsg.msg` | 控制模式、机械臂状态、运动/示教状态、轨迹号、错误码、各关节限位与通信状态 |
| `PosCmd.msg` | 末端 `x/y/z + roll/pitch/yaw + gripper + mode1/mode2` 命令 |
| `ObjectPos.msg` | 单个目标的 Header、三维点、宽、高 |
| `AllObjectPos.msg` | 多目标名称、三维点数组、宽度和高度数组 |
| `Enable.srv` | 请求使能/失能机械臂，返回操作结果 |
| `SetInterest.srv` | 请求设置关注目标名称，返回字符串结果 |
| `PlayText.srv` | 请求播放文本并指定是否同步，返回成功标志 |
| `GripperControl.action` | 目标夹爪角和宽度，返回成功标志及消息 |
| `PressButton.action` | 以唯一 YOLO 类别名触发按钮任务，返回成功/消息，并反馈状态、位置误差、角度误差和目标年龄 |

### 5.3 源码目录中存在、但当前 CMake 没有生成的接口

以下文件存在于 `srv/`：

- `GripperControl.srv`
- `GripperAction.action`

当前 `CMakeLists.txt` 的 `rosidl_generate_interfaces()` 已列出 `Enable.srv`、
`SetInterest.srv`、`PlayText.srv`、`action/GripperControl.action` 和
`action/PressButton.action`，但 `GripperControl.srv` 被注释，
`GripperAction.action` 也没有加入生成清单。

此外：

- `piper` 节点导入 `piper_msgs.srv.Enable`；
- `piper_vision` 导入 `piper_msgs.srv.SetInterest`；
- `piper_pbvs_control` 导入 `PressButton.action`、`SetInterest.srv` 和
  `PiperStatusMsg.msg`；
- `GripperAction.action` 放在 `srv/` 目录而不是 `action/`；
- `SetInterest.srv` 的请求 `name`、响应 `result` 已与视觉节点和初定位客户端一致。

仍需清理的是未生成的旧夹爪接口文件，并补充更完整的接口包依赖声明。

## 6. `piper`

路径：`src/piper`

### 6.1 作用

真实 Piper 单臂 ROS 2 驱动。节点通过 `piper_sdk.C_PiperInterface` 打开指定 CAN
接口，在独立线程中以 200 Hz 读取机械臂反馈，并接受关节角或末端位姿命令。

主可执行程序：

```text
piper_single_ctrl = piper.piper_ctrl_single_node:main
piper_trajectory_controller = piper.piper_trajectory_controller:main
piper_move_x = piper.piper_move_x:main
```

`piper_single_ctrl` 是真机 CAN 节点；`piper_trajectory_controller` 是 MoveIt
`FollowJointTrajectory` 到 `/joint_ctrl_single` 的真机桥；`piper_move_x` 是一次性
X 轴相对移动工具。

### 6.2 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `can_port` | `can0` | SocketCAN 接口名 |
| `auto_enable` | `false`（节点内） | 启动后是否自动尝试使能；单包 launch 默认 `true`，项目总入口显式传 `false` |
| `gripper_exist` | `true` | 是否控制夹爪 |
| `gripper_val_mutiple` | `1` | 夹爪行程倍率，限制到 0～10 |
| `tcp_offset_x` | `0.0` | J6 坐标系中的 TCP X 偏移，单位 m |
| `tcp_offset_y` | `0.0` | J6 坐标系中的 TCP Y 偏移，单位 m |
| `tcp_offset_z` | `0.1358` m | J6 坐标系中的标定 TCP Z 偏移，单位 m |

### 6.3 发布话题

| 话题 | 类型 | 含义 |
|---|---|---|
| `/joint_states_single` | `sensor_msgs/msg/JointState` | 六轴角度、速度和夹爪位置/力反馈 |
| `/joint_states` | `sensor_msgs/msg/JointState` | MoveIt 标准真实反馈，不修改六轴角度 |
| `/joint_states_raw` | `sensor_msgs/msg/JointState` | joint1～joint7真实反馈，用于初始化安全检查 |
| `/piper_command_enabled` | `std_msgs/msg/Bool` | 本次驱动进程是否会转发运动命令 |
| `/arm_status` | `piper_msgs/msg/PiperStatusMsg` | 控制器、运动、故障及关节状态 |
| `/end_pose` | `geometry_msgs/msg/Pose` | SDK 反馈的 J6 法兰位姿 |
| `/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 在 J6 位姿上叠加工具偏移后的 TCP 位姿，frame 为 `base_link` |

单位转换：

- 关节角：SDK 的“度 × 1000”转换到弧度；
- 末端位置：SDK 的微米转换到米；
- 末端欧拉角：SDK 的“度 × 1000”转换为四元数；
- 夹爪开度：SDK 微米值转换到米。

注意：节点创建的反馈关节名是 `joint0`～`joint6`，而机器人模型和控制回调使用的是
`joint1`～`joint7`。因此 `/joint_states_single` 不能无条件当作标准模型
`JointState` 使用，名称映射存在偏移。

### 6.4 订阅话题

| 话题 | 类型 | 行为 |
|---|---|---|
| `/pos_cmd` | `piper_msgs/msg/PosCmd` | 切到末端控制并调用 SDK `EndPoseCtrl`；同时可控制夹爪 |
| `/joint_ctrl_single` | `sensor_msgs/msg/JointState` | 依据关节名读取 joint1～joint6，位置控制机械臂；第 7 个位置控制夹爪 |
| `/enable_flag` | `std_msgs/msg/Bool` | 使能或失能机械臂和夹爪 |

当前主真机 launch 不再把 `joint_ctrl_single` 重映射为 `/joint_states`：
`/joint_states` 只发布反馈，`/joint_ctrl_single` 只接收真实轨迹控制器的命令，避免把
状态话题误当成电机目标。

`PosCmd` 中的 `mode1`、`mode2` 当前只被打印，没有参与运动模式选择；节点实际使用固定的
SDK 控制模式和速度参数。关节命令、末端命令也只有在节点内部使能标志为真时才会下发。

### 6.5 服务

`/enable_srv`，类型设计为 `piper_msgs/srv/Enable`：

- 请求 `enable_request=true`：循环发送使能并检查 6 个驱动器状态；
- 请求 `false`：发送失能；
- 超时 5 秒返回失败。

该服务使用由 `piper_msgs` 正常生成的 `Enable.srv`。

真实轨迹控制器还提供 `/initialize_arm`，类型为 `std_srvs/srv/Trigger`。它要求：

- 本次驱动进程已通过 `/enable_srv` 使能；
- `/joint_states_raw`、CAN位置反馈和六轴低速使能反馈均新鲜；
- joint2/joint3 仅在配置阈值内小幅越过零位，其他关节仍在URDF限位内；

满足条件后，控制器从真实起点生成 S 曲线插值，通过
`/joint_ctrl_single → JointCtrl` 直接移动到
`[-1.557051440, 0.234953236, -0.324353736, 0.013745872, 0.203885472, -0.018106872, 0.0020]`
的 joint1～joint7 Ready 姿态（joint1～joint6 单位为 rad，joint7 单位为 m）。该启动过程不进行路径避障规划；
到位后用 `/check_state_validity` 验证最终真实状态。初始化成功前，普通
FollowJointTrajectory目标会被拒绝。使能本身不会发布关节命令，因此调用初始化服务前
仍保留自由手动摆放行为。

由项目总入口 `all.launch.py` 启动时，初始化的当前默认参数为：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `initialization_speed_percent` | `20` | 初始化期间下发给 Piper 的速度百分比，控制器限制为 1%～20% |
| `initialization_duration` | `6.0 s` | 从当前姿态到 Ready 姿态的最短插值时长 |
| `initialization_max_step` | `0.006 rad` | 50 Hz 下的最大名义关节步长；位移较大时会自动延长总时长 |

服务请求本身是无参数的 `Trigger`，速度必须在启动时设置：

```bash
ros2 service call /initialize_arm std_srvs/srv/Trigger "{}"
```

若直接单独运行控制器而不经过总 launch，节点源码内的后备默认值仍为 12%、12 秒和
0.002 rad；推荐统一通过 `all.launch.py` 显式传入参数，避免启动方式不同造成速度差异。

### 6.6 `piper_move_x`

该节点从最新 `/tcp_pose` 出发，只改变 `base_link` X，保持 Y/Z、
姿态和夹爪开度。主要参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `distance_mm` | `65.0` mm | 有符号的 X 位移，范围 `-100～100 mm` |
| `enable_motion` | `false` | 物理运动开关 |
| `motion_algorithm` | `cartesian` | `cartesian` 直接末端控制，或 `moveit` 碰撞规划 |

`cartesian` 以 10 Hz、每步最大 2 mm 向 `/pos_cmd` 发布插值法兰目标，
同时检查命令源独占、硬件状态、反馈新鲜度、X 跟踪误差、横向漂移和
姿态漂移。`moveit` 向 `/move_action` 提交最终 TCP 约束，要求没有
`/pos_cmd` 发布者，且恰有一个 `/joint_ctrl_single` 发布者。两种模式均
以实测 TCP 验收终点；默认 dry-run 不下发运动。

### 6.7 launch

- `start_single_piper.launch.py`：只启动真机控制节点；
- `start_single_piper_rviz.launch.py`：先包含 `piper_description` 的模型展示 launch，
  再启动真机控制节点。

使用真机前必须先配置 SocketCAN，项目说明要求 1 Mbit/s，并安装 `piper_sdk`、
`python-can` 和 SciPy。

### 6.8 当前入口问题

`setup.py` 还声明了：

- `piper_ms_ctrl = piper.piper_start_ms_node:main`
- `piper_read_master = piper.piper_read_master_node:main`

但源码中没有这两个 Python 模块。`piper_single_ctrl`、
`piper_trajectory_controller` 和新增的 `piper_move_x` 均有实际实现。

## 7. `piper_description`

路径：`src/piper_description`

### 7.1 作用

这是所有 Piper 展示、MoveIt 和仿真的几何基础，包含：

- STL 网格：`base_link`、`link1`～`link8`、`gripper_base`
- 有夹爪和无夹爪的 URDF/Xacro
- Gazebo 专用 Xacro
- MuJoCo XML 模型
- RViz 配置
- 模型展示 launch

### 7.2 机器人结构

有夹爪模型包含：

- 世界到基座的固定关节；
- `joint1`～`joint6` 六个机械臂转动关节；
- `gripper_base`；
- 两个夹指滑动关节 `joint7`、`joint8`；
- 固定的 `tcp_link`，作为夹爪中心工具坐标系。

无夹爪模型保留六轴机械臂，不包含双指部分。

`joint8` 现为 `joint7` 的反向 mimic 关节，MoveIt 和真机标准
`/joint_states` 只需处理独立夹爪关节 `joint7`。

### 7.3 Gazebo 模型

`*_gazebo.xacro` 在模型中加入：

- 材质/摩擦等 Gazebo 标签；
- `ros2_control` 的 `gazebo_ros2_control/GazeboSystem`；
- 各关节 position command interface；
- position/velocity state interface；
- `gazebo_ros2_control/GazeboSystem` 硬件插件声明。

与已删除的 `piper_gazebo` 包解耦后，Xacro 已不再嵌入顶层
`libgazebo_ros2_control.so` 插件和指向 `piper_gazebo/config/*.yaml` 的路径。
因此它们当前是模型资源，不是完整的可启动 Gazebo 仿真系统。

### 7.4 MuJoCo 模型

| 文件 | 作用 |
|---|---|
| `piper_description.xml` | 完整六轴 + 双指模型，并带目标点自由关节 |
| `piper_no_gripper_description.xml` | 无夹爪完整六轴模型 |
| `piper_no_gripper_anthropomorphic_description.xml` | 仅前三轴“拟人臂”子模型，用于表驱动位置表生成 |
| `piper_no_gripper_wrist_description.xml` | 仅后三轴腕部子模型，用于姿态表生成 |

### 7.5 launch

四个展示 launch 分别覆盖有/无夹爪、URDF/Xacro，都会启动：

- `robot_state_publisher`
- `joint_state_publisher` 或 `joint_state_publisher_gui`
- RViz

当前文件命名与默认模型有交叉：

- `display_urdf.launch.py` 默认加载 `piper_description.xacro`
- `display_xacro.launch.py` 默认加载 `piper_description.urdf`

无夹爪版本则与名称一致。使用时应以源码中的默认 `model` 参数为准。

## 8. `piper_with_gripper_moveit`

路径：`src/piper_moveit/piper_with_gripper_moveit`

### 8.1 作用

MoveIt Setup Assistant 生成并经过修改的 Piper 配置。它面向带夹爪模型，并把
`tcp_link` 作为机械臂规划链末端，最符合当前 `grasp_server` 的约束定义。

### 8.2 规划组与语义

SRDF 中：

- `arm`：`base_link` → `tcp_link`；
- `gripper`：`gripper_base` → `link7`；
- `arm/zero`：六轴零位；
- `gripper/open`：`joint7=0.035`；
- `gripper/close`：`joint7=0`；
- 还配置了大量相邻或默认不碰撞 link 对。

KDL 同时为 `arm` 和 `gripper` 配置运动学插件。

### 8.3 ros2_control 与 MoveIt 控制器

该配置的 `piper.ros2_control.xacro` 使用 `mock_components/GenericSystem`，
标准 `demo.launch.py` 因此是 MoveIt 假硬件演示，不会直接访问 Piper CAN。

| 控制器 | 关节 |
|---|---|
| `arm_controller` | `joint1`～`joint6` |
| `gripper_controller` | 仅 `joint7` |
| `joint_state_broadcaster` | 发布 `/joint_states` |

默认控制器更新率为 500 Hz；MoveIt 默认速度和加速度缩放均为 0.1。

控制器只控制独立夹爪关节 `joint7`，`joint8` 由 URDF 中的反向 mimic
关系跟随，不再需要额外夹爪镜像节点。

### 8.4 launch

标准生成的 launch：

- `demo.launch.py`：完整演示，通常包含 RSP、move_group、RViz、ros2_control 和控制器；
- `move_group.launch.py`
- `moveit_rviz.launch.py`
- `rsp.launch.py`
- `spawn_controllers.launch.py`
- `static_virtual_joint_tfs.launch.py`
- `warehouse_db.launch.py`
- `setup_assistant.launch.py`

自定义 `piper_moveit.launch.py` 只启动：

- `move_group`
- RViz

并强制 `use_sim_time=true`，不启动 robot_state_publisher、controller_manager 或控制器。
它的用途是叠加在已经运行的 Gazebo 系统上，而不是独立运行。

当前真机入口是 `real_feedback_demo.launch.py`。它启动 RSP、move_group、
可选 RViz 和 `piper_trajectory_controller`，但不启动 GenericSystem、
`ros2_control_node` 或仿真 controller manager。`/joint_states` 由 Piper 真机驱动唯一提供。

### 8.5 传感器配置

`sensors_3d.yaml` 配置了 DepthImageOctomapUpdater，但图像话题仍是示例性的
`/head_mount_kinect/depth_registered/image_raw`，没有指向本项目 Orbbec 深度话题。
若要用于规划场景避障，需要改为实际相机话题并验证 TF。

## 9. `piper_moveit_config_v5`

路径：`src/piper_moveit_config_v5`

### 9.1 作用

另一套 MoveIt 配置。它与 `piper_with_gripper_moveit` 使用相同的
`piper_description`，但语义末端和夹爪控制方式不同。

### 9.2 与 `piper_with_gripper_moveit` 的关键差异

| 项目 | `piper_moveit_config_v5` | `piper_with_gripper_moveit` |
|---|---|---|
| `arm` 末端 | `link6` | `tcp_link` |
| arm 组定义 | 显式 joint1～joint6 + chain | `base_link` 到 `tcp_link` chain |
| gripper 组 | `joint7` + `joint8` | `gripper_base` 到 `link7`，实际控制 joint7 |
| 夹爪控制器 | joint7、joint8 | 仅 joint7 |
| 预设状态 | arm zero/ready，gripper open/close | arm zero，gripper open/close |
| 控制器更新率 | 100 Hz | 500 Hz |
| 单关节最大速度 | 约 1.61993 rad/s | arm 多数 5 rad/s，joint6 3 rad/s |
| 3D 传感器 YAML | 无 | 有 |
| 自定义 Gazebo MoveIt launch | 无 | 有 `piper_moveit.launch.py` |

### 9.3 使用边界

`grasp_server` 把位置/姿态约束的 `link_name` 固定为 `tcp_link`。因此若运行本 v5
配置，规划组 `arm` 的 tip 是 `link6`，与抓取节点的预期不一致。当前抓取链更应选
`piper_with_gripper_moveit`，或者统一修改 SRDF 和抓取节点参数。

此外，当前共用 `piper_description` 已将 `joint8` 设为 `joint7` 的 mimic，
而 v5 SRDF、ros2_control 和 MoveIt controller 仍把两者都当作独立可控关节。
这是当前 v5 配置的内部不一致，使用前应改为单主关节 mimic 语义。

该包的 `demo.launch.py` 等均调用 `moveit_configs_utils` 的标准生成函数，默认也是
GenericSystem 假硬件演示。

## 10. 已删除的 `piper_sim` 功能包

当前工作树已删除 `src/piper_sim/piper_gazebo` 和
`src/piper_sim/piper_mujoco`，`colcon list` 不再识别 `piper_gazebo` 或
`piper_mujoco`。因此，旧的 Gazebo launch、控制器 YAML、夹爪镜像脚本和
MuJoCo ROS 跟随节点均不再是当前项目入口。

`piper_description` 仍保留 Gazebo Xacro 和 MuJoCo XML，
`piper_control/ctrl_by_mujoco.py` 也仍可直接使用 MuJoCo Python API，
但如需恢复 ROS 仿真，需重新提供启动包、控制器配置和完整依赖声明。

## 11. `piper_tf`

路径：`src/piper_tf`

### 11.1 `handeye_static_tf`

这是当前眼在手上视觉链的关键节点。它解决的问题是：

- 手眼标定矩阵可能给出 `link6 <- camera_link`，也可能给出
  `link6 <- camera_color_optical_frame` 等其他标定 frame；
- Orbbec 驱动已经拥有并发布 `camera_link -> camera_color_optical_frame`；
- 若再直接把 optical frame 挂到 `link6`，会造成同一个 TF child 有两个父节点。

节点把矩阵统一解释为 `parent_frame <- calibrated_frame`。当
`calibrated_frame != camera_link_frame` 时，先等待相机驱动内部 TF，再计算：

```text
T(link6 <- camera_link)
  = T(link6 <- calibrated_frame)
  × inverse(T(camera_link <- calibrated_frame))
```

最终只发布：

```text
link6 -> camera_link
```

相机内部的 depth/color/IR optical frame 仍由相机驱动负责，TF 树保持单父节点结构。
当前 `handeye.yaml` 的标定结果明确使用 `tracking_base_frame=camera_link`，所以
`calibrated_frame=camera_link`，节点直接发布矩阵，不需要等待相机内部 frame 转换。

节点还会：

- 验证手眼矩阵必须恰好 16 个有限值；
- 验证最后一行为 `[0, 0, 0, 1]`；
- 通过 SVD 把旋转部分投影到合法的 SO(3)；
- 在相机 TF 就绪前按周期重试；
- 成功后发布一次静态 TF 并取消定时器。

参数：

| 参数 | 默认值 |
|---|---|
| `parent_frame` | `link6` |
| `camera_link_frame` | `camera_link` |
| `calibrated_frame` | `camera_link` |
| `handeye_matrix` | `config/handeye.yaml` 中的 4×4 矩阵 |
| `lookup_retry_period` | `0.5` 秒 |

`handeye_static_tf.launch.py` 加载 YAML，并允许覆盖三个 frame 名称。

### 11.2 `tf_transformer`

这是较早的目标点坐标转换节点：

- 订阅 `/camera_target_point`，类型 `PointStamped`；
- 查询消息自身 `frame_id` 到 `base_link` 的最新 TF；
- 发布 `/base_target_point`，类型 `PointStamped`。

新 YOLO 节点已经在内部直接变换到 `target_frame_id`，因此不再需要此节点完成同一转换；
但抓取节点仍依赖它产生的旧接口 `/base_target_point`。

## 12. `piper_vision`

路径：`src/piper_vision`

该包包含三条相对独立的视觉能力：YOLO11 Detect RGB-D 三维定位与按钮表面位姿、
ChArUco 位姿估计、VLM 语义地图。

### 12.1 `yolo_detect_3d`

节点名：`yolo_ros2`

核心流程：

1. 从必填绝对路径 `model_path` 加载 Ultralytics YOLO11 Detect `.pt` 模型，
   并拒绝非 detect 模型；
2. 用 `message_filters.ApproximateTimeSynchronizer` 同步彩色图和深度图，
   队列 3、时间容差 0.02 秒；
3. 单独读取 CameraInfo 内参和 optical frame，并用容量为 2 的工作队列只处理较新的
   同步 RGB-D 帧；
4. 校验彩色图和深度图尺寸一致，可按深度阈值把无效或过远背景替换成灰色；
5. 对 YOLO 检测框中心内缩区域的有效深度取中位数；
6. 使用 `cv2.undistortPoints` 从框中心像素和深度反投影到相机光学坐标；
7. 按图像时间戳查询 optical frame 到 `target_frame_id` 的 TF，发布目标点和尺寸；
8. 当 `interest` 是唯一类别时，在检测框外圈构造面板点云并做 RANSAC 平面拟合；
9. 将连续合格平面变换到 `base_link`，默认累计 5 帧；只有法向角度散布和法向
   偏移散布都合格时才锁定任务级面板；
10. 从当前 RGB 按钮框中心构造空间射线，与锁定面板求交，得到按钮表面中心；
11. 取朝向面板的轴作为工具 `+Z` 按压方向，构造按钮完整位姿；
12. 发布全部目标、关注目标、按钮位姿和标注图。

重要参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `model_path` | 无，必填 | YOLO11 Detect `.pt` 的绝对路径 |
| `device` | 空 | 留空由 Ultralytics 自动选择，也可设 `cpu`、`cuda:0` |
| `interest` | `all` | 精确模型类别名，或发布所有目标的 `all` |
| `depth_threshold` | `2.0` | 背景移除深度阈值，单位 m |
| `depth_scale` | `0.001` | 原始深度值到米的换算比例 |
| `box_roi_inset` | `0.25` | 检测框四边向内缩进比例 |
| `conf_threshold` | `0.7` | 检测置信度阈值 |
| `iou_threshold` | `0.45` | NMS IoU 阈值 |
| `bg_removal` | `false` | 是否移除深度阈值外背景；总入口也默认 false |
| `target_frame_id` | `base_link` | 三维目标输出坐标系 |
| `camera_frame_id` | 空 | 空时采用 CameraInfo 的 `frame_id` |
| `camera_info_topic` | `/camera/color/camera_info` | 相机参数 |
| `color_image_topic` | `/camera/color/image_raw` | 彩色图 |
| `depth_image_topic` | `/camera/depth/image_raw` | 必须与彩色图对齐的深度图 |

按钮平面质量参数包括 `plane_outer_scale=2.0`、
`plane_inner_scale=1.0`、`plane_ransac_threshold=0.003 m`、
`plane_min_points=100`、`plane_min_inlier_ratio=0.6`、
`plane_max_rms=0.004 m`、`plane_max_depth_deviation=0.03 m` 和
`plane_sample_step=3`。这是节点内置默认值；项目总入口针对近距离任务覆盖为
`plane_min_points=60`、`plane_sample_step=2`。面板锁定默认还要求连续 5 个候选
平面沿法向的偏移极差不超过 `5 mm`，最大法向角差不超过 `3°`。

发布：

| 话题 | 类型 | 含义 |
|---|---|---|
| `/piper_vision/pred_image` | `sensor_msgs/msg/Image` | 检测标注图 |
| `/piper_vision/target_point` | `piper_msgs/msg/ObjectPos` | 类别等于 `interest` 的目标 |
| `/piper_vision/all_object_points` | `piper_msgs/msg/AllObjectPos` | 所有通过阈值的目标 |
| `/piper_vision/button_pose` | `geometry_msgs/msg/PoseStamped` | 选定按钮的中心和按压方向，仅唯一类别且平面质量合格时发布 |

节点还创建 `/set_interest` 服务，类型为 `piper_msgs/srv/SetInterest`。
请求字段 `name` 必须是模型中的精确类别名或 `all`；未知类别会返回可用类别列表。
`piper_pbvs_control` 每次收到 `/press_button` Action 后，先用该服务选择目标类别。

`interest=all` 时会在 `/target_point` 发布所有有效检测，但不会生成
`/button_pose`；按钮位姿要求先选择唯一类别，避免多个按钮候选混淆。

最终按钮控制位置不是简单采用按钮框中心深度。面板锁定后，算法使用当前 RGB
中心射线与锁定平面求交；框内深度只用于普通 `ObjectPos` 和一致性诊断。这样可减少
近距离深度空洞、按钮凸起和反光对按压点的影响。切换 `/set_interest` 会立即清除
上一任务的面板锁，数字键与 `key_ok` 会分别重新采样。

`yolo_handeye.launch.py` 会同时启动：

- `piper_tf/handeye_static_tf`
- 在指定 Conda 环境中运行的 `piper_vision/yolo_detect_3d`

该 launch 要求传入 `model_path`，默认 Conda 环境为 `yolo11`，并把视觉、手眼
frame、图像话题和平面拟合参数都暴露为 launch 参数，是当前视觉链推荐入口。

### 12.2 `charuco_detector`

用途：检测 DICT_5X5_100 的 ChArUco 标定板并估计完整六自由度位姿。

默认标定板：

- 7 × 5 方格；
- 方格边长 0.035 m；
- ArUco marker 边长 0.026 m；
- 至少 4 个 ChArUco 角点。

输入：

- `/camera_info`
- `/image`

输出：

| 输出 | 类型 | 含义 |
|---|---|---|
| `/aruco_single/pose` | `geometry_msgs/msg/PoseStamped` | 标定板位姿 |
| `/aruco_single/result` | `sensor_msgs/msg/Image` | 经制 marker、角点和坐标轴的图像 |
| `marker_frame` TF | 动态 TF | 默认 child 为 `camera_marker` |

若设置 `reference_frame=base_link`，节点会查询 TF，把相机坐标中的板位姿转换到
`base_link` 后再发布。`charuco_single.launch.py` 默认同时启用手眼 TF，并把输入重映射到
Orbbec 彩色图和 CameraInfo。

### 12.3 `vlm_mapper_node`

设计用途：

1. 缓存最新彩色图；
2. 缓存 `/piper_vision/all_object_points`；
3. 触发时把图片上传到火山引擎 TOS；
4. 调用豆包视觉模型核验 YOLO 检测到的物体；
5. 把保留目标的二维坐标写入 `map/map.json`；
6. 或把一次识别详情写入 `records/<timestamp>.json`。

输入/触发：

- `/camera/color/image_raw`
- `/piper_vision/all_object_points`
- `/detection_trigger`（`std_msgs/msg/Empty`）
- `/piper_vision/map_capture`（`std_srvs/srv/Empty`）

外部依赖和环境变量：

- `volcenginesdkarkruntime`
- `tos`
- `ARK_API_KEY`
- `TOS_AK`
- `TOS_SK`

类型注解使用了 `Any`，但只从 `typing` 导入了 `Dict, List`。由于它出现在函数内的
实例属性注解中，CPython 当前不会求值该注解，因而不会仅因此触发运行时 `NameError`；
但静态类型检查和代码完整性上仍应补充 `Any` 导入。

另外，`piper_vision_api.py` 的语义地图路径硬编码为另一台机器的绝对路径：

```text
/home/lgw/study/ros_all/EmbodiedAIOS/map/map.json
```

应改为参数、包 share 路径或工作目录相对路径。

### 12.4 辅助模块

- `cv_tool.py`：像素点去畸变并按深度反投影；
- `yolo_geometry.py`：检查图像尺寸、检测框中心稳健深度、外围点云与 RANSAC 平面、
  optical 坐标、按压方向姿态等纯数学工具；
- `s3img.py`：上传本地图像到 TOS 并生成预签名 URL；
- `doubao.py`：创建豆包 Ark 客户端；
- `piper_vision_api.py`：从语义地图按中英文名称查询坐标。

### 12.5 入口与验证

当前安装入口均有对应源码：

- `yolo_detect_3d`
- `charuco_detector`
- `vlm_mapper_node`

`test_yolo_geometry.py` 覆盖图像配准尺寸、检测框边界、有效深度筛选、中心深度中位数、
外围平面拟合、按压轴姿态和 optical 坐标约定。运行 YOLO 主节点仍需要外部 Conda
环境中的 Ultralytics、OpenCV、NumPy、`rclpy` 和 `cv_bridge`。

## 13. `piper_grasp_control_by_ros`

路径：`src/piper_grasp_control_by_ros`

### 13.1 `grasp_server`

节点名：`grasp_server`

作用：保存最新抓取目标，在收到触发服务后构造 MoveIt `MoveGroup` Action 请求并执行。

输入：

| 名称 | 类型 | 规则 |
|---|---|---|
| `/base_target_point` | `geometry_msgs/msg/PointStamped` | 只使用位置；代码没有验证其 `frame_id` |
| `/base_target_pose` | `geometry_msgs/msg/PoseStamped` | 必须为 `base_link`；检查有限值并归一化四元数 |
| `/grasp_command` | `std_srvs/srv/Trigger` | 触发一次 MoveIt 规划和执行 |

目标选择采用“最后一个有效目标覆盖前一个”的策略：

- 收到 Pose 后清除旧 Point；
- 收到 Point 后清除旧 Pose；
- Pose 可指定完整方向；
- Point 路径用单位四元数，但把三个姿态容差都设为 π，相当于姿态基本自由。

发送给 MoveIt 的约束：

- 规划组：`arm`
- 末端 link：`tcp_link`
- 位置容差区域：0.01 m × 0.01 m × 0.01 m 的 BOX
- 显式姿态容差：各轴 0.1 rad
- Action：`/move_action`，类型 `moveit_msgs/action/MoveGroup`
- `plan_only=false`，允许 replan，replan delay 2 秒

节点还创建 `gripper_control` ActionClient，但抓取成功后自动调用夹爪的语句目前被注释，
所以“移动到目标”和“闭合夹爪”尚未串成完整抓取动作。

### 13.2 `gripper_controller_node`

节点名：`gripper_controller_node`

提供 `gripper_control` Action Server，类型 `piper_msgs/action/GripperControl`。

当前行为：

- 检查 `gripper_angle` 是否在 -0.785～0.785 rad；
- 检查 `gripper_width` 是否在 0.005～0.07 m；
- 只向 `/gripper_controller/joint_trajectory` 发布 joint7 的宽度；
- `gripper_angle` 对 joint6 的控制代码整体被注释，参数目前只参与范围检查；
- 发布后立即把 Action 标记成功，没有等待控制器实际到位。

### 13.3 当前逻辑风险

1. 目标点跳变过滤代码无效：即使进入“3 秒内跳变超过 0.1 m”的拒绝分支，函数末尾仍会
   无条件调用 `_accept_new_target()`。
2. `/grasp_command` 是同步 Service，但回调发出异步 MoveIt Action 后立即返回 response；
   Action 完成后的回调再修改已返回的 response，客户端通常无法得到真实执行结果。
3. 只保存一个 `_current_response`，并发触发会互相覆盖。
4. Point 输入没有像 Pose 输入那样检查 `frame_id`、有限值。
5. 夹爪自动闭合未接入抓取成功回调。

因此该包表达了完整设计方向，但抓取服务结果语义和末端执行阶段仍需要工程化完善。

## 14. `piper_pbvs_control`

路径：`src/piper_pbvs_control`

### 14.1 作用与状态机

这是当前电梯按钮任务的上层控制包，包含两个任务节点和一个独立归零节点：

- `piper_pbvs_controller`：完成一个指定按钮的视觉锁定、MoveIt 粗定位和可选推进；
- `elevator_sequence`：调用单键 Action 和 MoveIt 回位，把一位或两位楼层数字及确认键
  编排成完整选层序列；
- `joint_zero_return`：显式收到服务请求后，使用 MoveIt 将 joint1～joint7 依次归零。

包名和节点名仍保留 `pbvs`，但当前实现不执行连续图像闭环 PBVS。单键状态机为：

```text
IDLE
  → WAIT_TARGET
  → COARSE_APPROACH
  → X_ADVANCE（仅 distance_mm 非 0）
  → DONE / ABORT
  → IDLE
```

同一时间只接受一个非空 `target_name` 任务。取消、超时、目标丢失、
机械臂故障或实测到位验收失败都会进入 `ABORT`。成功后机械臂保持在最终
位姿，不由单键控制器自动回撤。完整序列会针对每一位楼层数字重复
`PRESS_NUMBER → RETURN_AFTER_NUMBER`，然后按下确认键：

```text
IDLE
  → PRESS_NUMBER
  → RETURN_AFTER_NUMBER
  → PRESS_OK
  → RETURN_AFTER_OK
  → DONE / ABORT
  → IDLE
```

完整序列中的 `RETURN_*` 是 MoveIt 关节空间回位，因此能在每一位数字键以及
`key_ok` 之后恢复统一观察姿态。

### 14.2 单键控制器 ROS 接口

| 方向 | 名称 | 类型 | 用途 |
|---|---|---|---|
| Action Server | `/press_button` | `piper_msgs/action/PressButton` | 以唯一 YOLO 类别名触发按钮任务 |
| 订阅 | `/piper_vision/button_pose` | `geometry_msgs/msg/PoseStamped` | `base_link` 下的实时按钮中心和按压方向 |
| 订阅 | `/tcp_pose` | `geometry_msgs/msg/PoseStamped` | 真机实测 TCP 位姿 |
| 订阅 | `/joint_states` | `sensor_msgs/msg/JointState` | MoveIt 六轴关节实测反馈 |
| 订阅 | `/arm_status` | `piper_msgs/msg/PiperStatusMsg` | 错误码、关节限位和通信故障 |
| Service Client | `/set_interest` | `piper_msgs/srv/SetInterest` | 把 Action 目标名同步给 YOLO |
| Service Client | `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | 加入面板和眼在手上相机碰撞体 |
| Action Client | `/move_action` | `moveit_msgs/action/MoveGroup` | 规划/执行粗定位和可选推进 |
| 发布 | `/pbvs/state` | `std_msgs/msg/String` | 当前状态 |
| 发布 | `/pbvs/desired_tcp_pose` | `geometry_msgs/msg/PoseStamped` | 调试用目标 TCP |

Action 反馈包含当前状态、位置误差、角度误差和目标数据年龄。

### 14.3 一次初定位的控制过程

1. 调用 `/set_interest` 选择 Action 指定的唯一 YOLO 类别；
2. 等待默认 3 个位置和姿态离散度均合格的按钮位姿 B0；
3. 向 MoveIt 场景加入 0.6 × 1.2 × 0.02 m 面板碰撞盒，并可把
   0.10 × 0.04 × 0.04 m 相机盒附着到 `link6`；
4. 按按压轴后退 `coarse_standoff=0.08 m`，再沿按钮局部 +X 应用
   `coarse_horizontal_offset`，得到固定 MoveIt 目标 C0；
5. dry-run 只规划一次并结束；真机模式最多执行首次 + 3 次重试，
   每次都使用同一个 B0/C0；
6. 以 `/tcp_pose` 实测 T0 验收：法向误差绝对值不超过 10 mm，
   面板切平面误差模长必须落在配置闭区间内，姿态也必须合格；
7. `distance_mm != 0` 时，从实测 T0 保持姿态并使用 MoveIt 沿
   `base_link X` 或锁定面板法向移动 `-100～100 mm`，再以实测 TCP 验收；
8. 进入 `DONE`，保持当前位置。

### 14.4 姿态策略与验收约束

默认 `orientation_mode=preserve_current_roll`：Action 开始时读取实测 TCP 姿态，
只用最短旋转把工具 `+Z` 对齐视觉按压轴，保留任务起始滚转。
`world_up` 则直接使用视觉节点根据面板法线构造的完整姿态。
当前两种模式都只用于 MoveIt 位姿约束，不再转换为 EndPoseCtrl 欧拉角。

关键默认约束：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enable_motion` | `false` | false 时 MoveIt 仅规划 |
| `coarse_standoff` | `0.08 m` | TCP 相对按钮的法向后退距离 |
| `coarse_horizontal_offset` | `0.026` m | 按钮局部 +X 补偿，面向面板时正值向左 |
| `coarse_lateral_error_min/max` | `0.009 / 0.019 m` | 实测面板切平面误差的闭区间；小于下限或大于上限均不通过 |
| `coarse_axial_tolerance` | `0.01 m` | 实测法向距离误差上限 |
| `coarse_correction_attempts` | `0` | 首次执行后的重试数，真机总计最多 4 次 |
| `distance_mm` | `65.0` mm | 粗定位后的有符号推进距离，范围 `±100 mm` |
| `x_advance_axis_mode` | `base_x` | `base_x` 沿基座 X；`panel_normal` 沿视觉按压轴 |
| `moveit_velocity_scaling_factor` | `0.07` | 粗定位、校正及推进的 MoveIt 速度缩放为 7% |
| `moveit_acceleration_scaling_factor` | `0.07` | 粗定位、校正及推进的 MoveIt 加速度缩放为 7% |
| `stable_sample_count` | `3` | 稳定目标样本数 |
| `target_acquire_timeout` | `5.0 s` | 等待稳定按钮位姿的超时 |
| `tcp_feedback_timeout` | `0.5 s` | TCP 反馈新鲜度要求 |

节点持续检查 `err_code`、六轴角度限位和六轴通信状态。`distance_mm` 只定义目标
位移，不包含接触力、按钮行程或按钮灯判据，不应解读为有反馈保证的真实按压。

### 14.5 完整电梯序列 `elevator_sequence`

节点提供 `/run_elevator_sequence` Action Server，复用同一个
`piper_msgs/action/PressButton` 类型。目标规范化规则为：

- 空字符串：读取 `floor_number` 参数，默认 `1`；
- 一位或两位数字（如 `3`、`10`）：按字符转换成一个或两个 `key_N`；
- 带 `key_` 前缀的一位或两位数字（如 `key_3`、`key_10`）：移除前缀后按位转换；
- 其他字符串：拒绝 Action goal。

严格执行过程：

1. 按楼层数字顺序调用 `/press_button` 定位/推进对应 `key_N`；
2. 每完成一位数字，都用 `/move_action` 回到 `home_joint_positions`；
3. 所有数字完成后，调用 `/press_button` 定位/推进 `key_ok`；
4. 再次用 `/move_action` 回到初始关节位；
5. 所有按键和回位全部成功后才返回成功。

例如 10 楼的严格顺序为：

```text
key_1 → home → key_0 → home → key_ok → home
```

默认 home 为：

```text
[-1.557051440, 0.234953236, -0.324353736, 0.013745872,
 0.203885472, -0.018106872] rad
```

真机模式下，MoveIt 回位成功后还会检查新鲜 `/joint_states`，要求六轴最大误差小于
`home_joint_tolerance + 0.001 rad`。如果单键任务在 `WAIT_TARGET` 识别阶段失败，
序列直接中止；如果已进入 `COARSE_APPROACH` 或 `X_ADVANCE` 后失败，编排器会先尝试
`RECOVERY_HOME`，然后把完整任务标记失败。正常回位失败时不会继续按下一个按钮。
取消、超时或状态不确定时停止后续步骤，不额外启动恢复运动。

序列接口：

| 方向 | 名称 | 类型 | 用途 |
|---|---|---|---|
| Action Server | `/run_elevator_sequence` | `piper_msgs/action/PressButton` | 一位/两位数字逐键回位、OK、最终回位任务 |
| Action Client | `/press_button` | `piper_msgs/action/PressButton` | 调用单键任务 |
| Action Client | `/move_action` | `moveit_msgs/action/MoveGroup` | 每次按键后的关节回位和失败恢复回位 |
| 订阅 | `/joint_states` | `sensor_msgs/msg/JointState` | 真机回位验收 |
| 订阅 | `/pbvs/state` | `std_msgs/msg/String` | 判断子任务是否已进入运动阶段 |
| 发布 | `/elevator_sequence/state` | `std_msgs/msg/String` | 完整任务状态 |

### 14.6 launch 与测试

`elevator_press.launch.py` 包含 `piper_vision/yolo_handeye.launch.py`，并同时启动
`piper_pbvs_controller` 与 `elevator_sequence`；
它不负责启动相机、Piper 驱动或完整 MoveIt 系统。`model_path` 是必填参数，
唯一物理运动开关 `enable_motion` 默认为 false。

`control_math.py` 封装四元数/旋转、位姿误差、稳定姿态平均、
按压轴/面板水平偏移、粗定位误差分解、验收区间和 X 距离校验。
测试覆盖这些纯数学逻辑、控制器诊断输出、楼层目标规范化、回位目标构造、关节反馈
验收、完整序列顺序、失败阻断和运动阶段失败后的回位恢复分支。

### 14.7 七关节归零 `joint_zero_return`

`joint_zero_return` 提供 `/return_all_joints_zero`（`std_srvs/srv/Trigger`）。服务被显式
调用后，先规划/执行 arm 组的 joint1～joint6 到 `0 rad`，再规划/执行 gripper 组的
joint7 到 `0 m`；真机模式最后使用新鲜 `/joint_states` 验收。默认六轴容差为
`0.01 rad`，joint7 容差为 `0.003 m`。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `enable_motion` | `false` | false 时只验证两段归零规划，不向真机下发 |
| `zero_velocity_scaling_factor` | `0.10` | 两段 MoveIt 归零规划的速度缩放为 10% |
| `zero_acceleration_scaling_factor` | `0.10` | 两段 MoveIt 归零规划的加速度缩放为 10% |
| `zero_timeout` | `30.0 s` | 每个 MoveIt 归零任务的等待超时 |

总 launch 默认 `start_joint_zero_return=true`，因此会提供该服务，但不会自动执行归零：

```bash
ros2 service call /return_all_joints_zero std_srvs/srv/Trigger "{}"
```

服务请求不携带速度。需要在启动 `all.launch.py` 时通过
`zero_velocity_scaling_factor` 和 `zero_acceleration_scaling_factor` 设置。

## 15. `piper_launch`

路径：`src/piper_launch`

### 15.1 作用

这是当前电梯按键任务的总启动包，本身没有控制节点。它用统一参数条件性包含：

1. `orbbec_camera/gemini_330_series.launch.py`
2. `piper/start_single_piper.launch.py`
3. `piper_with_gripper_moveit/real_feedback_demo.launch.py`
4. `piper_pbvs_control/elevator_press.launch.py`
5. `piper_pbvs_control/joint_zero_return.launch.py`

第 4 个入口会继续启动视觉、手眼 TF、单键控制器和完整序列编排器，因此总启动后
会提供 `/press_button`、`/run_elevator_sequence` 和 `/return_all_joints_zero`。归零节点
由第 5 个入口启动，服务只有收到显式请求才执行。

默认面向 Gemini 335L、`can0` 和 Conda 环境 `yolo11`。YOLO 模型
由 `model_path` 或环境变量 `PIPER_MODEL_PATH` 提供，两者都未设置时为空路径。
`start_camera`、`start_piper`、`start_moveit`、`start_pbvs` 可分别关闭，以复用已经
运行的组件。

### 15.2 安全默认与主要参数

`all.launch.py` 默认：

- `auto_enable=false`
- `enable_motion=false`
- `use_rviz=true`
- `enable_handeye_tf=true`
- `orientation_mode=preserve_current_roll`
- `distance_mm=0.0`
- `start_joint_zero_return=true`
- `initialization_speed_percent=20`
- `initialization_duration=6.0`
- `initialization_max_step=0.006`
- `moveit_velocity_scaling_factor=0.07`
- `moveit_acceleration_scaling_factor=0.07`
- `zero_velocity_scaling_factor=0.10`
- `zero_acceleration_scaling_factor=0.10`
- `coarse_lateral_error_min/max=0.009/0.019 m`

因此默认启动不会自动使能真机，控制器只请求 MoveIt 规划。
MoveIt 入口同时启动真实 `piper_trajectory_controller`，只有显式设置
`enable_motion=true` 且已完成 `/initialize_arm` 后才会下发轨迹。
`distance_mm=0` 时，即使允许粗定位运动也不会继续向按钮推进。当前已无
`enable_press` 参数。

总 launch 还透传 CAN 接口、夹爪倍率、TCP 偏移、相机名/序列号/USB 口、YOLO 设备、
手眼 TF、平面拟合质量、初始化速度、MoveIt 定位/回位/归零速度、粗定位水平补偿、
横向验收区间、校正次数和 `distance_mm` 等参数。模型路径不再硬编码到旧工作区。

### 15.3 `diagnostic.launch.py`

该入口不会启动或控制硬件，而是并行运行限定时长的 `ros2 topic hz`，默认测量 8 秒：

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/arm_status`
- `/tcp_pose`
- `/piper_vision/button_pose`

它可以确认关键数据是否持续到达，但不验证 TF 数值、MoveIt 规划质量、碰撞场景或
真实运动精度。

### 15.4 当前部署边界

总启动已覆盖相机、视觉、TF、MoveIt、单键/完整电梯任务控制和真机驱动，但 MoveIt 配置仍使用
MoveIt Simple Controller Manager，而不是标准真机 `ros2_control SystemInterface`。
粗接近已经通过真实 `FollowJointTrajectory` 服务端下发，并由真机 `/joint_states`
闭环反馈；长期产品化仍可把 Piper SDK 封装为标准硬件接口。

## 16. MoveIt、任务控制器与真机的关系

这部分容易混淆，单独总结：

1. 旧 `grasp_server` 把整个机械臂运动目标交给 MoveIt；
2. 新按钮任务的 `COARSE_APPROACH` 和可选 `X_ADVANCE` 都交给 MoveIt；
3. 完整序列的 `RETURN_AFTER_NUMBER`、`RETURN_AFTER_OK` 和必要的
   `RECOVERY_HOME` 也交给 MoveIt；
4. MoveIt 最终向 `arm_controller/follow_joint_trajectory` 发送轨迹；
5. 真机入口不启动 `mock_components/GenericSystem`、模拟
   `joint_state_broadcaster` 或模拟轨迹控制器；
6. `piper_trajectory_controller` 订阅真机 `/joint_states`，按轨迹时间插值后
   发布完整七关节 `/joint_ctrl_single`；
7. `piper` 真机节点订阅 `/joint_ctrl_single` 并调用厂商 SDK，同时发布唯一的
   MoveIt 标准反馈 `/joint_states`。
8. `/joint_states` 和 `/joint_states_raw` 都保留真实六轴角度，不再向MoveIt发布
   虚假零位；后者用于初始化过程独立检查原始反馈新鲜度。

因此当前按钮运动只有一条下发路径：

```text
粗定位 / 可选推进 / 关节回位：
MoveIt → FollowJointTrajectory → piper_trajectory_controller
       → /joint_ctrl_single → piper JointCtrl → CAN
```

轨迹控制器同时用第二个只读 SocketCAN socket 检查 `0x2A5`～`0x2A7`
关节反馈和 `0x261`～`0x266` 六轴使能反馈的新鲜度，拒绝重复
`/joint_states` 发布者，并实现轨迹限位、线性插值、路径/终点误差、取消保持和
Action 错误传播。MoveIt 配置的通用默认速度/加速度缩放为 10%，但当前 PBVS 粗定位、
校正、panel-normal 推进及序列 home 回位都在目标中显式使用 7%；七关节归零默认显式使用
10%。`trajectory_speed_percent` 是 Piper 轨迹桥的硬件侧速度百分比，默认 10%，与 MoveIt
缩放不是同一个参数。六轴终点误差需各自小于 0.01 rad 并连续稳定 5 个反馈周期。
控制器每次启动默认关闭真实轨迹门，只有显式调用 `/initialize_arm`，按总 launch 默认
20% 速度、至少 6 秒且单步不超过 0.006 rad 到达 Ready，并通过最终状态检查后才开放。
单键控制器仍用 `/tcp_pose` 验证实测 TCP，并用 `/arm_status` 做故障保护。

按钮任务不再在 MoveIt 关节模式与 `/pos_cmd` 末端模式之间切换，
但桥接方案仍不是标准 `ros2_control SystemInterface`。MoveIt 真机轨迹已不再
经过 FakeSystem。

若要长期稳定用于真实任务，建议把 Piper SDK 封装成
`hardware_interface::SystemInterface`，让 MoveIt、controller_manager 和真机共享标准
`joint_states` 与 FollowJointTrajectory 闭环。

## 17. 推荐的启动组合

以下命令按当前源码入口整理。所有真机操作都应先确认 CAN、TF、初始关节状态、碰撞场景
和物理急停。

### 17.1 构建与加载工作区

首次克隆仓库或修改 `src/` 中的 Python、YAML、launch 默认值后，在工作区根目录构建：

```bash
cd /home/xie/wzl/elevate_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install_wzl/setup.bash
```

若只是修改 `ros2 launch ... 参数:=值` 的本次启动值，不需要重新构建，停止并重新启动
launch 即可。每个新终端仍需加载 `/opt/ros/humble/setup.bash` 和
`install/setup.bash`。

### 17.2 只启动真实机械臂

```bash
source install_wzl/setup.bash
ros2 launch piper start_single_piper.launch.py \
  can_port:=can0 \
  auto_enable:=false \
  gripper_exist:=true \
  gripper_val_mutiple:=2
```

注意先正确配置 CAN，并在安全工作区内操作。

### 17.3 Orbbec + 手眼 TF + YOLO 三维定位

终端 1：

```bash
ros2 launch orbbec_camera <实际型号>.launch.py depth_registration:=true
```

终端 2：确保 `robot_state_publisher` 已发布 `base_link -> link6`。

终端 3：

```bash
ros2 launch piper_vision yolo_handeye.launch.py \
  model_path:=/absolute/path/to/best.pt \
  device:=cuda:0 \
  target_frame_id:=base_link \
  interest:=key_3
```

输出目标在 `/piper_vision/target_point`；选择唯一类别且平面拟合合格时，还会输出
`/piper_vision/button_pose`。

### 17.4 电梯按钮单键安全 dry-run

```bash
ros2 launch piper_launch all.launch.py \
  model_path:=/absolute/path/to/best.pt \
  auto_enable:=false \
  enable_motion:=false
```

发送一个模型中确实存在的按钮类别：

```bash
ros2 action send_goal /press_button \
  piper_msgs/action/PressButton \
  "{target_name: 'key_3'}" --feedback
```

默认流程应为 `WAIT_TARGET → COARSE_APPROACH → DONE → IDLE`。它会调用 MoveIt
做 `plan_only`，但不会自动使能机械臂，也不会下发真机轨迹。

### 17.5 完整电梯序列 dry-run

总系统启动后，发送空目标会使用 `floor_number`（默认 1）：

```bash
ros2 action send_goal /run_elevator_sequence \
  piper_msgs/action/PressButton \
  "{target_name: ''}" --feedback
```

也可只覆盖本次楼层：

```bash
ros2 action send_goal /run_elevator_sequence \
  piper_msgs/action/PressButton \
  "{target_name: '3'}" --feedback
```

完整顺序为 `key_3 → home → key_ok → home`。在 `enable_motion=false` 下，两次单键
和两次回位都只规划不执行。仅启动节点或设置 `floor_number` 不会自动开始任务。

两位楼层按数字逐位执行，例如 10 楼：

```bash
ros2 action send_goal /run_elevator_sequence \
  piper_msgs/action/PressButton \
  "{target_name: '10'}" --feedback
```

顺序为 `key_1 → home → key_0 → home → key_ok → home`。

机械臂已使能且总 launch 使用 `enable_motion:=true` 时，可先初始化到 Ready：

```bash
ros2 service call /initialize_arm std_srvs/srv/Trigger "{}"
```

需要把 joint1～joint7 全部归零时，显式调用：

```bash
ros2 service call /return_all_joints_zero std_srvs/srv/Trigger "{}"
```

前者速度由 `initialization_speed_percent`、`initialization_duration` 和
`initialization_max_step` 控制；后者速度由 `zero_velocity_scaling_factor` 和
`zero_acceleration_scaling_factor` 控制。这些参数均在启动 `all.launch.py` 时设置，
不能通过 `Trigger "{}"` 请求临时传入。

### 17.6 单独测试 `piper_move_x`

```bash
ros2 run piper piper_move_x --ros-args \
  -p motion_algorithm:=cartesian \
  -p distance_mm:=65.0 \
  -p enable_motion:=false
```

预览通过后，确认其他命令发布者均已停止、机械臂已使能且物理急停可用，
再设置 `enable_motion:=true`。如使用 `motion_algorithm:=moveit`，需先启动
MoveIt 真机入口和轨迹桥，并完成 `/initialize_arm`。


### 17.7 MoveIt FakeSystem 独立演示

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

这只是 GenericSystem 假硬件。不应与 `real_feedback_demo.launch.py` 或真机总启动
同时运行，否则会出现重复的 MoveIt、TF 或 `/joint_states` 来源。

### 17.8 ChArUco 位姿检测

先启动 Orbbec，再运行：

```bash
ros2 launch piper_vision charuco_single.launch.py
```

默认输出 `base_link` 中的 `/aruco_single/pose`，前提是机器人 TF 与手眼 TF 连通。

## 18. 非 ROS 辅助目录：`piper_control`

路径：`src/piper_control`

虽然它不是 colcon 功能包，但仍保留了多种 Piper 控制后端的实验性抽象。

### 18.1 控制抽象

`CtrlBase` 定义统一接口：

- 重置；
- 发送一步控制；
- 设置/增量设置关节；
- 获取关节、夹爪、末端位置和四元数；
- 控制夹爪；
- 渲染。

`CtrlByMujoco`：

- 使用新版 `mujoco` Python API；
- 直接从 `src/piper_description/mujoco_model` 加载 XML；
- 写 actuator control；
- 读取 `ee_site` 位姿；
- 可离屏渲染。

当前 `set_gripper()` 内调用 `self.ctrl.set_joint()`，但类中没有 `self.ctrl`，应调用自身的
关节控制方法；这条路径目前有实现错误。

`CtrlByPiperSDK`：

- 使用 `C_PiperInterface_V2("can0")`；
- 支持关节控制和末端位姿控制；
- 处理 SDK 单位转换；
- 自动使能、复位、夹爪控制和失能；
- 析构时会尝试复位并断开真实机械臂。

`CtrlByROS`：

- 订阅 `/joint_states`；
- 发布 arm/gripper JointTrajectory；
- 但没有继承 `CtrlBase`，也没有初始化 `joint_num`，当前不完整。

### 18.2 表驱动控制

`table_driven_control` 把机械臂拆为：

- 前三轴拟人臂：由末端/腕部位置查 KDTree；
- 后三轴腕部：由相对旋转向量查 KDTree。

`gen_table.py` 用 MuJoCo 穷举关节角并建立 SciPy KDTree；
`table_driven_ctrl.py` 尝试组合两张表得到六轴关节角。

当前 demo 仍包含开发者绝对路径、调试渲染、运行时缺少顶层导入等问题，属于研究原型，
并非可直接部署的 IK 求解器。

## 19. 当前源码中最需要优先处理的问题

按对系统闭环的影响排序：

### P0：真机控制安全与状态闭环

1. 保持 `/joint_ctrl_single` 和 `/pos_cmd` 的单一命令所有权；
   `piper_move_x` 已做发布者计数检查，但系统级仍宜增加显式仲裁。
2. 当前真机 MoveIt 已使用真实 `/joint_states` 和带 CAN 新鲜度/跟踪误差保护的
   FollowJointTrajectory 桥；需继续验收取消、失能、通信中断和跟踪超差时的硬件行为。
3. 长期将 Piper SDK 封装为 `ros2_control SystemInterface`，减少自定义轨迹桥和
   controller manager 之间的语义差异。
4. 进一步消除兼容话题 `joint0`～`joint6` 与模型 `joint1`～`joint8` 的命名歧义。

### P1：完成单键与完整电梯序列真机验收

1. 按相机 → 手眼 TF → 平面法线 → MoveIt dry-run → 真机初定位
   → 可选推进 → 回位 → `key_ok` → 再回位的顺序验收。
2. 当前 `coarse_lateral_error_min/max=9～19 mm` 是闭区间，会拒绝小于 9 mm 的横向误差；
   继续真机标定时应核实保留“误差下限”是否符合末端结构的实际补偿需求。
3. 保持节点、YAML、子 launch 和总 launch 的 9～19 mm 默认区间一致，修改后同时检查
   `ros2 param dump /piper_pbvs_controller` 的运行时生效值。
4. 如要恢复真实接触按压，需重新设计力/触觉检测、合规末端、位移/速度限制和回撤策略；
   不应把当前 `distance_mm` 直接当作按压功能。

### P1：整理旧抓取与实验入口

1. 若继续保留旧 `grasp_server`，应增加 YOLO `ObjectPos` 适配或改为直接订阅，
   并参数化末端 link、规划组和 MoveIt 配置。
2. 修复其跳变过滤、异步 Service 返回、并发 response 覆盖、Point 校验和夹爪闭环。
3. 删除或补齐失效 console entry point：
   `piper_ms_ctrl`、`piper_read_master`。

### P2：清理配置与可移植性

1. 选定并保留一套主 MoveIt 配置，或清晰命名“TCP 单指控制版”和“link6 双指版”。
2. `piper_launch` 已改用 `model_path` / `PIPER_MODEL_PATH`；仍需移除
   `piper_vision_api` 和表驱动控制中的开发者绝对路径。
3. 补齐各包 `package.xml` 的真实运行依赖。
4. 清理未生成的 `GripperControl.srv` 和误放在 `srv/` 的
   `GripperAction.action`，或正式纳入接口生成。
5. 将语义地图路径、VLM 模型名和云存储参数改成 ROS 参数。
6. 把 Octomap 深度话题从 Kinect 示例改为实际 Orbbec 话题。

## 20. 包间依赖关系总结

源码 `package.xml` 明确声明的工作区内部依赖只有：

```text
orbbec_camera
└── orbbec_camera_msgs

piper
└── piper_msgs

piper_with_gripper_moveit
├── piper_description
└── piper

piper_moveit_config_v5
└── piper_description

piper_vision
├── piper_msgs
└── piper_tf

piper_grasp_control_by_ros
└── piper_msgs

piper_pbvs_control
├── piper_msgs
├── piper_vision
├── piper_tf
└── piper_with_gripper_moveit

piper_launch
├── orbbec_camera
├── piper
├── piper_pbvs_control
└── piper_with_gripper_moveit

```

但运行时依赖比清单更丰富：

- `piper` 的 ROS/Python 依赖已在 manifest 中覆盖主要接口、NumPy、SciPy 和
  `python-can`，但厂商 `piper_sdk` 仍需按包 README 额外安装；
- `piper_pbvs_control` 还依赖 MoveIt 服务/Action、NumPy，以及由总系统提供的真机反馈；
- `piper_vision` 还依赖 Ultralytics、OpenCV、NumPy、message_filters、
  `cv_bridge` 和火山引擎 SDK 等；
- `piper_launch` 已声明四个直接包含的主包，但完整运行仍依赖它们的所有传递依赖、
  Conda 环境、YOLO 权重、CAN 和相机硬件。

所以 `colcon graph` 只能反映 manifest 中已声明的依赖，不能完整代表本工作区真实运行图。

## 21. 总结

这个工作区已经具备一套具身机械臂系统的主要模块：

- Orbbec RGB-D 硬件接入；
- Piper 真机 CAN 控制；
- URDF、Gazebo Xacro 和 MuJoCo XML 模型资源；
- MoveIt 运动规划；
- 手眼 TF；
- YOLO11 Detect RGB-D 三维目标定位和面板法线估计；
- ChArUco 位姿检测；
- 视觉引导的 MoveIt 单键粗定位 + 可选双模式推进 Action；
- 一位/两位楼层数字逐键回位、再按 `key_ok` 并最终回位的完整电梯任务 Action；
- 需要显式调用的 joint1～joint7 MoveIt 归零服务；
- 安全默认的电梯按钮总启动入口；
- MoveIt 抓取任务原型；
- VLM 语义地图；

相机 → 手眼 TF → YOLO 按钮位姿 → `/press_button` → MoveIt → 真机反馈验收的
单键链路已经闭合，`/run_elevator_sequence` 又在其上完成每位楼层数字、确认键和每次
按键后的关节回位编排。系统默认禁用自动使能和物理运动。真机反馈已通过
`/joint_states` 接入 MoveIt/TF，MoveIt 轨迹也已通过受保护的轨迹桥进入 CAN。

当前实现的边界必须明确：默认只做规划；真机模式默认只完成按钮前粗定位；配置
`distance_mm` 后虽可继续推进，但没有接触力、按钮行程或按钮灯闭环，因此不能保证
按钮已触发。系统也不处理电梯到站、门状态或移动底盘进出电梯。Gazebo/MuJoCo ROS
功能包已删除。下一步应聚焦真机分阶段验收、粗定位容差语义统一、接触安全闭环和
系统级命令所有权。旧抓取和 VLM 模块仍包含明显的开发中代码。
