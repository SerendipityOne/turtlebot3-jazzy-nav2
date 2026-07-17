# TurtleBot3 Jazzy SLAM 与 Nav2 工作区

这是一个面向教学与验证的 ROS 2 Jazzy 工作区，集成 TurtleBot3、Gazebo Sim、Cartographer、Slam Toolbox 和 Nav2。它可用于在 `turtlebot3_house` 场景中完成建图、保存地图、定位和自主导航的完整流程。

## 功能

- 使用 Gazebo Sim 启动 TurtleBot3 仿真。
- 使用 Cartographer 构建二维栅格地图。
- 使用键盘或 Xbox 手柄遥控驱动仿真机器人完成建图。
- 使用 Nav2 在内置或保存的地图上完成定位和导航。

## 学习文档

- [Slam Toolbox 建图到 Nav2 优化全流程](docs/slam-toolbox-nav2-workflow.md)：从统一 rosbag 建图、地图验收到分阶段导航优化的可执行 Runbook。
- [2D SLAM 与 Nav2 优化学习指南](docs/2d-slam-nav2-learning-guide.md)：Cartographer/Slam Toolbox A/B、SmacPlanner2D 和 MPPI 的原理与配置。
- [实验记录模板](docs/experiment-record.md)：记录输入、参数校验值、固定 waypoint 和量化结果。

## 环境要求

- Ubuntu 24.04
- ROS 2 Jazzy
- 已安装 `git` 并已将本项目放入 ROS 2 工作区根目录

在工作区根目录执行以下命令安装构建工具和工作区声明的依赖：

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep

# 仅首次使用 rosdep 时执行
sudo rosdep init
rosdep update

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 构建

在工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

后续每个新终端都需要加载工作区并设置机器人型号：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

本文档默认使用 `burger`。`my_nav2.launch.py` 使用本项目的 `my_burger.yaml` 参数，因此导航流程应保持该型号。

## 启动 Gazebo

终端 1：加载环境后启动 `turtlebot3_house` 仿真场景。

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

该启动文件会启动 Gazebo Sim、生成机器人、建立 ROS-Gazebo 桥接，并发布仿真时钟。

## SLAM 建图

保持 Gazebo 运行，并在其他终端重复执行“后续每个新终端”的环境初始化命令。

终端 2：启动 Cartographer 和建图 RViz。

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

终端 3：启动键盘遥控，控制机器人探索场景。

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

使用 `w`、`a`、`s`、`d`、`x` 控制运动；空格键或 `s` 可立即停车。

### 使用 Xbox 手柄遥控

Xbox 手柄可替代键盘遥控。保持 Gazebo 和 Cartographer 运行，在终端 3 启动手柄节点：

```bash
ros2 launch turtlebot3_teleop xbox_teleop.launch.py
```

该启动文件默认使用 Linux 手柄索引 `0`，并启动 `joy_node`、`teleop_twist_joy` 和速度平滑器。若手柄是第二个设备，指定其索引：

```bash
ros2 launch turtlebot3_teleop xbox_teleop.launch.py device_id:=1
```

默认配置要求按住按钮 `4` 才会发送速度指令；按钮 `5` 启用高速模式，线速度和角速度分别使用轴 `1`、轴 `3`。手柄轴和按钮编号由 Linux `joy` 驱动提供，可能因设备而异。键盘和手柄都会发布 `/cmd_vel`，只能启动其中一种。

完成探索后，在终端 4 保存当前 `/map`：

```bash
mkdir -p "$HOME/maps"
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/tb3_house"
```

该命令会生成 `$HOME/maps/tb3_house.yaml` 和 `$HOME/maps/tb3_house.pgm`。

## Nav2 自主导航

导航和 SLAM 都会发布地图与相关 TF，因此启动 Nav2 前请停止 Cartographer 和键盘遥控。Gazebo 必须保持运行。

### 使用项目内置地图

终端 2：启动本项目的 Nav2 配置和 RViz。

```bash
ros2 launch turtlebot3_navigation2 my_nav2.launch.py use_sim_time:=true
```

该命令默认加载 `turtlebot3_navigation2/map/tb3_house.yaml`。在 RViz 中使用 **2D Pose Estimate** 设置机器人初始位姿，再使用 **Nav2 Goal** 设置导航目标。

### 使用自己保存的地图

将 `map` 参数替换为保存得到的 YAML 文件的绝对路径：

```bash
ros2 launch turtlebot3_navigation2 my_nav2.launch.py \
  use_sim_time:=true \
  map:="$HOME/maps/tb3_house.yaml"
```

如果初始定位偏差较大，请在 RViz 中重新设置初始位姿后再发送目标点。

## 运行流程

```text
Gazebo Sim
  -> Cartographer SLAM
  -> 键盘遥控探索
  -> 保存地图
  -> Nav2 定位与导航
```

SLAM 与 Nav2 是前后两个阶段，不应同时运行。

## 目录说明

```text
src/
├── turtlebot3/             # TurtleBot3 本体、Cartographer、Nav2 与键盘遥控
└── turtlebot3_simulations/ # Gazebo Sim 场景、机器人模型和桥接配置
```

## 常见问题

- `TURTLEBOT3_MODEL` 未设置：在启动任何 TurtleBot3 相关节点前执行 `export TURTLEBOT3_MODEL=burger`。
- 仿真时间异常：确认 Cartographer 和 Nav2 命令均包含 `use_sim_time:=true`。
- Nav2 无法定位：确认 Gazebo 使用的场景与地图匹配，并在 RViz 中设置正确的初始位姿。

## 上游项目与许可证

本项目基于 [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) 和 [TurtleBot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) 开发。上游组件继续遵循各自目录中的许可证；本项目新增内容遵循根目录的 Apache License 2.0。
