# Piper_Moveit2

[EN](README(EN).md)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

> 注：安装使用过程中出现问题可查看第5部分

## 1 安装Moveit2

1）二进制安装，[参考链接](https://moveit.ai/install-moveit2/binary/)

```bash
sudo apt install ros-humble-moveit*
```

2）源码编译方法，[参考链接](https://moveit.ai/install-moveit2/source/)

## 2 使用环境

安装完Moveit2之后，需要安装一些依赖

```bash
sudo apt-get install ros-humble-control* ros-humble-joint-trajectory-controller ros-humble-joint-state-* ros-humble-gripper-controllers ros-humble-trajectory-msgs
```

若系统语言区域设置不为英文区域，须设置

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

## 3 moveit控制真实机械臂

### 3.1 开启piper_ros

按照[piper_ros](../../README.MD#1-安装方法)配置完后

```bash
cd ~/piper_ros
source install/setup.bash
bash can_activate.sh can0 1000000
```

开启控制节点

```bash
ros2 launch piper start_single_piper.launch.py gripper_val_mutiple:=2
```

### 3.2 moveit2控制

开启moveit2

```bash
cd ~/piper_ros
conda deactivate # 若无conda环境可去除此行
source install/setup.bash
```

#### 3.2.1 有夹爪运行

```bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

![piper_moveit](../../asserts/pictures/piper_moveit.png)

可以直接拖动机械臂末端的箭头控制机械臂

调整好位置后点击左侧MotionPlanning中Planning的Plan&Execute即可开始规划并运动

## 4 可能遇见的问题

### 4.1 运行demo.launch.py时报错

报错：参数需要一个double，而提供的是一个string
解决办法：
终端运行

```bash
echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
source ~/.bashrc
```

或在运行launch前加上LC_NUMERIC=en_US.UTF-8
例如

```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch piper_moveit_config demo.launch.py
```
