# piper_pbvs_control

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


该包使用按钮视觉位姿执行 Piper 的 MoveIt 粗定位。保留原有节点、Action
和话题名称以兼容现有调用，但不再执行 PBVS、按压或自动回撤：

```text
WAIT_TARGET → COARSE_APPROACH [→ X_ADVANCE] → DONE → IDLE
```

包内另有独立的 `elevator_sequence` 编排节点。它不改变上述单键控制逻辑，
而是调用已有 `/press_button` 和 MoveIt，完成：

```text
数字键 → 初始关节位 → key_ok → 初始关节位
```

## 启动与调用

先启动 Piper 实机驱动和可控制实机的 MoveIt `move_group`，并确保只有一套
手眼 TF 发布器。随后启动视觉和粗定位控制器：

```bash
source install_wzl/setup.bash
ros2 launch piper_pbvs_control elevator_press.launch.py \
  model_path:=/absolute/path/to/best.pt
```

默认 `enable_motion=false`，只进行 MoveIt `plan_only`，不会控制机械臂。
Action 接口保持不变：

```bash
ros2 action send_goal /press_button piper_msgs/action/PressButton \
  "{target_name: 'key_3'}" --feedback
```

完整电梯任务使用另一个 Action。空目标使用 `floor_number` 参数（默认 1）：

```bash
ros2 action send_goal /run_elevator_sequence \
  piper_msgs/action/PressButton \
  "{target_name: ''}" --feedback
```

也可以只覆盖本次任务，例如按数字 3 后再按 OK：

```bash
ros2 action send_goal /run_elevator_sequence \
  piper_msgs/action/PressButton \
  "{target_name: '3'}" --feedback
```

两位楼层会按每位数字依次执行，并在每次按键后回位。例如楼层 10：

```bash
ros2 action send_goal /run_elevator_sequence \
  piper_msgs/action/PressButton \
  "{target_name: '10'}" --feedback
```

严格顺序为 `key_1 → home → key_0 → home → key_ok → home`。

只有收到该 Action 请求才会执行序列；仅启动节点或设置参数不会运动。目标只
接受一位或两位数字、对应的 `key_` 前缀形式或空字符串。数字键识别失败时，
现有单键 Action 会在发出 MoveIt 运动前中止。若单键任务已经进入 MoveIt 初定位或
按压移动后返回失败，编排节点会先尝试回到 `home_joint_positions`，然后将
整个任务标记失败；恢复回位失败或数字键正常回位失败时均不会继续按 OK。

默认回位参数为：

```yaml
floor_number: 1
home_joint_positions: [-1.557051440, 0.234953236, -0.324353736,
  0.013745872, 0.203885472, -0.018106872]
home_joint_tolerance: 0.01
```

### 独立的任意姿态七关节归零节点

`joint_zero_return` 是一个选择性启动的新节点，不会修改或替代任何按钮任务节点。
它提供 `/return_all_joints_zero` Trigger 服务，从机械臂当前姿态开始，严格按以下
顺序执行：

1. 通过 MoveIt `arm` 组将 `joint1`～`joint6` 规划或执行到 `0 rad`；
2. 通过 MoveIt `gripper` 组将 `joint7` 规划或执行到 `0 m`；
3. 真机模式下使用新鲜 `/joint_states` 验收七个关节。

该节点不会由现有 `all.launch.py` 自动启动。先进行无运动规划验证：

```bash
ros2 launch piper_pbvs_control joint_zero_return.launch.py \
  enable_motion:=false
```

另一个终端发送归零请求：

```bash
ros2 service call /return_all_joints_zero std_srvs/srv/Trigger "{}"
```

确认 MoveIt 规划、真机反馈、驱动使能和 `/initialize_arm` 均正常后，才可显式开启
真机运动：

```bash
ros2 launch piper_pbvs_control joint_zero_return.launch.py \
  enable_motion:=true
```

独立状态话题为 `/joint_zero_return/state`。该节点不订阅按钮状态，也不会调用
`/press_button`。

默认归零速度和加速度缩放均为 `0.10`。如需临时覆盖，可直接给节点传参：

```bash
ros2 launch piper_pbvs_control joint_zero_return.launch.py \
  enable_motion:=true \
  zero_velocity_scaling_factor:=0.10 \
  zero_acceleration_scaling_factor:=0.10
```

可在总启动命令中覆盖，例如：

```bash
ros2 launch piper_launch all.launch.py \
  floor_number:=3 \
  home_joint_positions:="[-1.557051440, 0.234953236, -0.324353736, 0.013745872, 0.203885472, -0.018106872]"
```

回位使用 MoveIt 关节目标并在真机模式下通过 `/joint_states` 验收，不调用只
允许首次初始化的 `/initialize_arm`。取消、超时或状态不确定时立即停止后续
步骤，不自动发出额外恢复运动。

完成 dry-run 后，真机粗定位可显式启用运动。若实机需要向左补偿
`30 mm`：

```bash
ros2 launch piper_pbvs_control elevator_press.launch.py \
  model_path:=/absolute/path/to/best.pt \
  enable_motion:=true \
  coarse_horizontal_offset:=0.026 \
  distance_mm:=65.0
```

正补偿表示面向电梯时向左，负补偿表示向右。补偿范围限制为
`-0.05～+0.05 m`，默认 `0 m`。

## 粗定位和验收

按钮位姿来自 `/piper_vision/button_pose`，使用 `base_link` 表达；位置为
按钮表面中心，姿态 `+Z` 为按压方向。默认姿态策略
`orientation_mode:=preserve_current_roll` 会保持任务开始时的末端滚转，只
将 TCP `+Z` 对齐按压方向；`world_up` 使用视觉输出的完整四元数。

MoveIt 将 `tcp_link` 移到按钮按压方向反向 `80 mm` 的 C0。真机到达后，
控制器使用 B0 和实测 T0 验收：

- 法向距离误差不超过 `±10 mm`。
- 面板切平面横向误差模长必须位于闭区间 `25～35 mm`。
- 姿态误差不超过 MoveIt 姿态容差的 `1.5` 倍。

每次失败都使用同一个 B0 和 C0 再执行 MoveIt。默认
`coarse_correction_attempts=3`，即首次加 3 次校正，最多执行 4 次；任意
一次成功便立即结束校正。`distance_mm=0`时保持在实测 T0；
非零且 `enable_motion=true`时，再从 T0 沿 `base_link` X 移动指定
距离。该动作复用 MoveIt 碰撞规划，只保证终点的 X 偏移，不保证
中间路径严格沿 X 直线。X 移动实测验收允许 `5 mm` 位置误差和
`0.075 rad`（约 `4.30°`）姿态误差。任务始终不执行 PBVS，不发布
`/pos_cmd`。

主要参数：

- `coarse_standoff`：法向粗定位距离，默认 `0.08 m`。
- `coarse_horizontal_offset`：水平补偿，默认 `0 m`。
- `coarse_lateral_error_min` / `coarse_lateral_error_max`：横向验收下限和
  上限，默认 `0.009 / 0.019 m`。
- `coarse_axial_tolerance`：法向距离误差，默认 `0.01 m`。
- `coarse_correction_attempts`：首次定位后的校正次数，默认 `3`。
- `moveit_velocity_scaling_factor` / `moveit_acceleration_scaling_factor`：
  粗定位和 panel-normal 按压的 MoveIt 速度/加速度缩放，默认均为 `0.07`
  （7%），并显式写入每个规划请求。
- `distance_mm`：粗定位成功后的按压位移，允许 `-100～100 mm`，默认
  `0 mm` 不移动；正值靠近按钮，负值远离按钮。
- `x_advance_axis_mode`：默认 `panel_normal`，沿视觉锁定的面板按压轴移动；
  `base_x` 沿固定 `base_link +X`。

## 接口与诊断

- Action：`/press_button`
- 完整序列 Action：`/run_elevator_sequence`
- 输入：`/piper_vision/button_pose`、`/tcp_pose`、`/joint_states`、
  `/arm_status`
- MoveIt：`/move_action`、`/apply_planning_scene`
- 调试：`/pbvs/state`、`/pbvs/desired_tcp_pose`
- 序列调试：`/elevator_sequence/state`

成功时会打印 B0、未补偿 C0、补偿后 C0、实测 T0、MoveIt 执行误差、
实测水平位移、法向距离和横向误差区间。例如：

```text
【阶段成功】MoveIt 粗定位（COARSE_APPROACH）：...（第 1/4 次）
【定位诊断】实测相对未补偿目标水平位移=+30.00 mm（正值向左，负值向右）
【阶段成功】MoveIt 按压移动（X_ADVANCE）：...
【粗定位状态】任务完成（DONE）
```

推荐检查：

```bash
ros2 topic echo /piper_vision/button_pose
ros2 topic echo /pbvs/desired_tcp_pose
ros2 topic echo /tcp_pose
ros2 topic echo /pbvs/state
```

任一机械臂错误码、关节限位、通信异常、TCP 反馈超时或 MoveIt 失败都会
中止任务。真机运行时应确保路径无障碍并由操作员全程手持急停。
