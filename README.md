# TurtleBot3 Jazzy：SLAM、Nav2 与具身目标导航

这是一个面向学习和验证的 ROS 2 Jazzy 工作区。在 `turtlebot3_house` Gazebo 仿真中，它串联了二维 SLAM、Nav2 自主导航、RGB-D 目标定位、自然语言任务解析，以及连续 SmolVLA 局部目标决策。

当前默认机器人是带 RGB-D 相机的 `waffle_pi_cam`。基础导航使用 SmacPlanner2D + MPPI；具身实验目前只验证了彩色球目标。

```text
自然语言指令
  -> LLM 解析任务阶段
  -> YOLO11n + RGB-D 确认目标
  -> SmolVLA 生成局部目标
  -> Nav2 规划、避障并控制底盘
```

## 主要功能

- 使用 Gazebo Sim 运行 TurtleBot3 House 仿真。
- 使用 Slam Toolbox 在线建图，或使用 Cartographer 进行对照实验。
- 使用键盘或 Xbox 手柄遥控机器人探索环境。
- 使用 Nav2、SmacPlanner2D 和 MPPI 完成定位与自主导航。
- 使用 YOLO11n、RGB-D、LLM 和 Nav2 执行自然语言找球任务。
- 使用 Gazebo Oracle 数据训练连续 SmolVLA，并通过 Shadow/Active 模式进行安全验收。

## 已验证范围

- Ubuntu 24.04、ROS 2 Jazzy、Gazebo Sim。
- `turtlebot3_house` 场景与 `waffle_pi_cam` 仿真机器人。
- 具身感知当前是 ball-only，支持红、绿、蓝、黄球。
- SmolVLA 输出局部目标，不直接发布 `/cmd_vel`；路径规划和底盘控制仍由 Nav2 负责。

模型、checkpoint、数据集和 `.env` 均被 Git 忽略，不会随仓库克隆。仓库只提供代码、配置、`.env.example` 和可复现流程；运行具身实验前需要按文档准备本地资产。

## 快速开始

### 1. 克隆仓库

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
git clone https://github.com/SerendipityOne/turtlebot3-jazzy-nav2.git "$TB3_WS"
cd "$TB3_WS"
```

### 2. 安装依赖

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep
```

仅在系统尚未初始化 rosdep 时执行：

```bash
sudo rosdep init
rosdep update
```

安装工作区声明的依赖：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
cd "$TB3_WS"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 构建

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
cd "$TB3_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

后续每个新终端先执行：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export TURTLEBOT3_MODEL=waffle_pi_cam
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
```

## Gazebo、SLAM 与 Nav2

以下命令分别在新终端运行；每个终端都要先执行上面的环境初始化。

### 1. 启动 Gazebo

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

该启动文件会启动 Gazebo、生成机器人、建立 ROS-Gazebo 桥接并发布仿真时钟。

### 2. 启动 SLAM

推荐使用 Slam Toolbox 在线建图：

```bash
ros2 launch turtlebot3_navigation2 slam_toolbox.launch.py use_sim_time:=true
```

如需运行 Cartographer 对照实验：

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

两种 SLAM 不要同时启动。

### 3. 遥控探索

键盘遥控：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Xbox 手柄遥控：

```bash
ros2 launch turtlebot3_teleop xbox_teleop.launch.py
```

默认使用手柄索引 `0`。如果手柄是第二个设备：

```bash
ros2 launch turtlebot3_teleop xbox_teleop.launch.py device_id:=1
```

### 4. 保存地图

```bash
export MAP_DIR="$HOME/maps"
mkdir -p "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/tb3_house"
```

命令会生成 `$HOME/maps/tb3_house.yaml` 和 `$HOME/maps/tb3_house.pgm`。

### 5. 启动 Nav2

启动 Nav2 前停止 SLAM 和遥控节点，保持 Gazebo 运行：

```bash
ros2 launch turtlebot3_navigation2 my_nav2.launch.py use_sim_time:=true
```

默认加载项目内置地图。使用自己保存的地图时：

```bash
export MAP_YAML="$HOME/maps/tb3_house.yaml"
ros2 launch turtlebot3_navigation2 my_nav2.launch.py \
  use_sim_time:=true \
  map:="$MAP_YAML"
```

在 RViz 中使用 **2D Pose Estimate** 设置初始位姿，再通过 **Nav2 Goal** 或 waypoint 工具发送导航目标。

## 具身找球与 SmolVLA

仓库提供两条运行路径：

- 固定视点找球：LLM 解析指令，YOLO11n 与 RGB-D 定位目标，Nav2 搜索并停在目标 `0.8 m` 范围内。
- 连续 SmolVLA：LLM 给出任务阶段，SmolVLA 根据当前图像和机器人状态生成局部目标，Nav2 负责实际执行。

开始前从 `.env.example` 创建本地 `.env`，并准备文档要求的 YOLO ONNX、SmolVLM 基座和 SmolVLA checkpoint。不要提交 API Key 或本地模型。

```bash
cp .env.example .env
```

完整的多终端启动、数据门禁、训练和 Shadow/Active 验收命令见下方文档。

## 文档

| 文档 | 内容 |
| --- | --- |
| [Slam Toolbox 建图到 Nav2 优化全流程](docs/slam-toolbox-nav2-workflow.md) | 在线建图、地图验收、AMCL 与 Nav2 参数优化。 |
| [2D SLAM 与 Nav2 优化学习指南](docs/2d-slam-nav2-learning-guide.md) | Cartographer/Slam Toolbox 对比、rosbag 复现、SmacPlanner2D 与 MPPI。 |
| [实验记录模板](docs/experiment-record.md) | 固定输入、waypoint、参数和量化结果的记录方法。 |
| [具身找球运行手册](docs/embodied-object-navigation-workflow.md) | ball-only 感知、语言解析、固定视点搜索和 Nav2 接近闭环。 |
| [分层 SmolVLA 找球与导航流程](docs/hierarchical-smolvla-navigation-workflow.md) | 连续 VLA 数据、门禁、训练、Shadow 和 Active 验收。 |

## 目录结构

```text
src/
├── turtlebot3/                      # TurtleBot3 本体、Cartographer、Nav2 与遥控
├── turtlebot3_embodied_interfaces/  # 具身导航 action 接口
├── turtlebot3_embodied_navigation/  # RGB-D 感知、LLM/VLA 接入与任务编排
└── turtlebot3_simulations/          # Gazebo 场景、机器人模型和数据生成器

docs/                                # 学习、实验和运行文档
tools/vla/                           # VLA 数据校验、转换、训练和策略服务工具
```

## 常见问题

- 找不到 TurtleBot3 模型：确认当前终端已设置 `TURTLEBOT3_MODEL=waffle_pi_cam`。
- 仿真时间异常：确认 SLAM、Nav2 和具身节点均启用了 `use_sim_time:=true`。
- Nav2 无法定位：确认 Gazebo 场景与地图匹配，并在 RViz 中重新设置初始位姿。
- 具身节点找不到模型：本地 `artifacts/`、`outputs/` 和 `datasets/` 不受 Git 跟踪，需要按对应工作流准备。

## 上游项目与许可证

本项目基于 [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) 和 [TurtleBot3 Simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) 开发。上游组件继续遵循各自目录中的许可证；本项目新增内容遵循根目录的 [Apache License 2.0](LICENSE)。
