# TurtleBot3 2D SLAM 与 Nav2 优化学习指南

如果需要从固定 rosbag 开始，按命令逐步完成 Slam Toolbox 建图和 Nav2 优化，请优先使用 [Slam Toolbox 建图到 Nav2 优化全流程](./slam-toolbox-nav2-workflow.md)。本文保留算法原理、Cartographer 对照和完整插件参数。

本文用于指导你在现有项目上逐步完成以下升级：

1. 为当前 Cartographer、NavFn 和 DWB 建立可比较的运行基线。
2. 接入 Slam Toolbox，并保留 Cartographer 作为对照组。
3. 将 NavFn 全局规划器替换为 SmacPlanner2D。
4. 将 DWB 局部控制器替换为 MPPI。
5. 使用同一个原始 rosbag 比较两种 SLAM，并使用相同场景和目标点比较导航表现。

每一阶段都包含原理、修改内容、运行命令和验收条件。不要一次完成所有修改；上一阶段验证通过后再继续。

实验过程中统一填写 [实验记录](./experiment-record.md)。该文档用于保存环境、Git 提交、参数校验值、bag 元数据、固定目标和测试结果，避免只凭运行时印象比较算法。

## 1. 目标、范围与最终结构

目标环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim 和 `ros_gz`
- TurtleBot3 Burger
- 2D 激光雷达 `/scan`
- `turtlebot3_house` 仿真场景

本指南只优化二维建图和二维导航。RGB-D、三维点云、RTAB-Map、自主探索和自定义 Nav2 插件留到二维系统稳定之后。

完成指南后，相关文件结构如下：

```text
src/turtlebot3/turtlebot3_navigation2/
├── launch/
│   ├── my_nav2.launch.py
│   └── slam_toolbox.launch.py          # 本指南新增
├── param/
│   ├── my_burger.yaml                  # 改为 Smac2D + MPPI
│   ├── my_burger_dwb_navfn.yaml        # 原配置基线
│   └── slam_toolbox.yaml               # 本指南新增
├── CMakeLists.txt
└── package.xml                         # 补充运行依赖
```

现有 `CMakeLists.txt` 已安装整个 `launch/` 和 `param/` 目录，不需要为新增文件增加安装规则。

## 2. 开始前准备

### 2.1 创建学习分支

先确认工作区干净：

```bash
cd ~/GitHub/tb3_jazzy_ws
git status --short
```

如果没有输出，创建分支：

```bash
git switch -c feature/slam-nav2-upgrade
```

如果存在未提交修改，先确认这些修改的用途，不要直接覆盖或丢弃。

### 2.2 加载环境

每个新终端都执行：

```bash
cd ~/GitHub/tb3_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

### 2.3 检查依赖

```bash
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_smac_planner
ros2 pkg prefix nav2_mppi_controller
```

三个命令都应该输出 `/opt/ros/jazzy` 下的路径。如果有包缺失，再安装对应 Jazzy 软件包：

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2
```

重新加载 ROS 环境后再次检查。不要在依赖检查失败时继续修改配置。

### 2.4 录制统一的 SLAM 输入 bag

要公平比较 Cartographer 和 Slam Toolbox，两种算法必须接收同一组 `/scan`、里程计和 TF。最简单的方法是先只运行 Gazebo 和遥控节点，录制不包含任何 SLAM 输出的原始输入 bag。

终端 1，启动 Gazebo：

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

终端 2，启动遥控：

```bash
ros2 launch turtlebot3_teleop xbox_teleop.launch.py
```

此时不要启动 Cartographer、Slam Toolbox 或 Nav2。检查节点和话题：

```bash
ros2 node list
ros2 topic list --include-hidden-topics | sort
```

节点列表中不应出现 `cartographer_node`、`slam_toolbox`、`amcl`、`planner_server` 或 `controller_server`。

终端 3，定义本次 bag 路径并开始录制：

```bash
export BAG_ROOT="$HOME/GitHub/tb3_jazzy_ws/bags/"
export SLAM_INPUT_BAG="$BAG_ROOT/slam_input_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BAG_ROOT"

ros2 bag record \
  --use-sim-time \
  --storage mcap \
  --output "$SLAM_INPUT_BAG" \
  --topics \
  /clock /scan /odom /tf /tf_static /joint_states /cmd_vel
```

按预定探索路线完成一整套数据采集，最后回到起点附近。在录包终端按 `Ctrl+C`，等待 rosbag 输出关闭完成后再关闭 Gazebo。

检查 bag：

```bash
printf 'SLAM_INPUT_BAG=%s\n' "$SLAM_INPUT_BAG"
ros2 bag info --sort name "$SLAM_INPUT_BAG"
```

确认至少包含 `/clock`、`/scan`、`/odom`、`/tf` 和 `/tf_static`，并且 `/scan`、`/odom` 消息数大于零。把 bag 的完整路径、时长和消息数写入 [实验记录](./experiment-record.md)。本指南当前使用的 bag 已记录在 [本次实验记录](./experiment-record-20260714-152204.md)。

原始输入 bag 不记录 `/map`、`/amcl_pose` 或 SLAM 节点输出。否则重放时可能同时出现录制的 `map -> odom` 和当前算法发布的 `map -> odom`。

## 3. 阶段一：使用 rosbag 建立 Cartographer 基线

“更好”必须由相同输入下的结果决定。从本阶段开始，所有 SLAM 建图都使用 2.4 节录制的同一个原始 bag，不再为不同算法分别启动 Gazebo 和遥控。

### 3.1 固定输入 bag

本次实验使用已经录制完成的 bag：

```bash
export SLAM_INPUT_BAG="$HOME/GitHub/tb3_jazzy_ws/bags/slam_input_xxx"
test -f "$SLAM_INPUT_BAG/metadata.yaml"
ros2 bag info --sort name "$SLAM_INPUT_BAG"
```

后续每个新终端都要重新导出相同路径。不要修改、裁剪或覆盖该 bag。

### 3.2 使用 Cartographer 重放建图

停止 Gazebo、遥控、Slam Toolbox、AMCL 和 Nav2。终端 1 启动 Cartographer，并使用仿真时间：

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
```

终端 2 从 bag 开头播放 SLAM 所需输入：

```bash
export SLAM_INPUT_BAG="$HOME/GitHub/tb3_jazzy_ws/bags/slam_input_xxx"
ros2 bag play "$SLAM_INPUT_BAG" \
  --clock 50 \
  --rate 1.0 \
  --delay 2 \
  --topics /scan /odom /tf /tf_static
```

`--clock 50` 为使用 `use_sim_time` 的节点发布回放时钟。这里只播放建图需要的传感器、里程计和 TF，不回放 `/cmd_vel`。

播放期间重点观察：

- 回到已知区域后墙体是否出现明显双边或重影。
- 闭环前后地图是否发生大幅跳变。
- 房间和走廊是否闭合。
- `/map` 是否持续更新。

保存基线地图：

```bash
mkdir -p "$HOME/GitHub/tb3_jazzy_ws/maps/replay"
ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/GitHub/tb3_jazzy_ws/maps/replay/cartographer_house"
```

保存后停止 Cartographer，确认 `cartographer_node` 已退出。不要让它与后续 Slam Toolbox 同时发布 `map -> odom`。

将输入 bag、播放参数和地图路径填写到 [本次实验记录](./experiment-record-20260714-152204.md)。

### 3.3 使用 Cartographer 地图记录 Nav2 基线

rosbag 可以公平复现 SLAM 输入，但不能替代导航的闭环运动。导航测试需要重新启动 Gazebo，并显式加载刚才生成的地图。

终端 1：

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

终端 2，启动当前 Nav2：

```bash
ros2 launch turtlebot3_navigation2 my_nav2.launch.py \
  use_sim_time:=true \
  map:="$HOME/GitHub/tb3_jazzy_ws/maps/replay/cartographer_house.yaml" \
  initial_pose_x:=0.0 \
  initial_pose_y:=1.0
```

1. 在 RViz 的 Navigation 2 面板进入 `Waypoint mode`。

2. 使用 Nav2 Goal 依次设置多个点。
3. 点击 `Save WPs`，保存 YAML。文件包含每个点的 `position x/y/z` 和 `orientation x/y/z/w`。
4. 测试时点击 `Load WPs` 加载同一文件。
5. 根据路线选择导航模式：

| 路线与实验目标 | 导航模式 |
| --- | --- |
| 非闭环、无折返，要求连续平滑通过中间点 | `Start Nav Through Poses` |
| 终点接近起点、存在重叠或折返，要求逐点严格到达 | `Start Waypoint Following` |

`NavigateThroughPoses` 的 feedback 直接包含当前位置、剩余距离、恢复次数和剩余点数。`FollowWaypoints` 自身只反馈当前点序号，但它会逐点调用 `NavigateToPose`；同时记录两者的 feedback 后，可以获得每个点的完整导航信息。

如果机器人启动时已经位于最后一个目标的 `xy_goal_tolerance` 和 `yaw_goal_tolerance` 内，`Start Nav Through Poses` 可能直接报告到达。当前闭环 `waypoint2.yaml` 应选择 `Start Waypoint Following`。

开始导航前录包：

```bash
export NAV_BAG="$HOME/GitHub/tb3_jazzy_ws/bags/slam_input_xxx/cartographer_nav_waypoints"

ros2 bag record \
  --use-sim-time \
  --storage mcap \
  --include-hidden-topics \
  --output "$NAV_BAG" \
  --topics \
  /clock /map /amcl_pose /tf /tf_static /cmd_vel /scan \
  /plan /local_plan \
  /local_costmap/costmap /global_costmap/costmap \
  /follow_waypoints/_action/feedback \
  /follow_waypoints/_action/status \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status \
  /navigate_through_poses/_action/feedback \
  /navigate_through_poses/_action/status
```

### 3.4 基线记录表

在自己的实验记录中填写：

| 指标 | Cartographer + NavFn + DWB |
| --- | --- |
| 10 个目标成功数 | 10 |
| 碰撞次数 | 未记录，无法判定 |
| 触发恢复行为次数 | 0 |
| 明显振荡次数 | 2 次终点调姿；0 次持续振荡 |
| 平均到达时间 | 16.051 s（纯导航） |
| 最大位置误差 | 0.144 m（waypoint2） |
| 最大角度误差 | 约 0.204 rad（最后一条 feedback） |

数据来自 `bags/slam_input_20260714_152204/cartographer_nav_waypoints`。最终位姿和误差取每个 `NavigateToPose` 目标成功前的最后一条 feedback，因此最大角度误差是近似值。本次 bag 未录制碰撞接触话题或 Gazebo ground truth，不能据此确认碰撞次数。

### 3.5 保存原 Nav2 参数

在修改 `my_burger.yaml` 前建立可直接运行的基线副本：

```bash
cp src/turtlebot3/turtlebot3_navigation2/param/my_burger.yaml \
  src/turtlebot3/turtlebot3_navigation2/param/my_burger_dwb_navfn.yaml
```

检查两个文件相同：

```bash
diff -u \
  src/turtlebot3/turtlebot3_navigation2/param/my_burger.yaml \
  src/turtlebot3/turtlebot3_navigation2/param/my_burger_dwb_navfn.yaml
```

预期没有输出。

建议提交第一个检查点：

```bash
git add src/turtlebot3/turtlebot3_navigation2/param/my_burger_dwb_navfn.yaml
git commit -m "Add Nav2 baseline configuration"
```

## 4. 阶段二：接入 Slam Toolbox

### 4.1 为什么先使用同步模式

Slam Toolbox 使用二维扫描匹配和位姿图优化完成建图。同步模式按顺序处理所有接收到的扫描，适合当前规模较小、算力充足的 Gazebo 场景，也便于建立可重复基线。

如果以后在大地图中发现扫描队列持续积压，再评估 `online_async_launch.py`；当前阶段不同时改变运行模式和算法参数。

### 4.2 创建 Slam Toolbox 参数文件

创建：

```text
src/turtlebot3/turtlebot3_navigation2/param/slam_toolbox.yaml
```

写入以下完整配置：

```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: true
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.02
    map_update_interval: 5.0
    resolution: 0.05
    restamp_tf: false
    min_laser_range: 0.12
    max_laser_range: 3.5
    minimum_time_interval: 0.5
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5
    check_min_dist_and_heading_precisely: false
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    min_pass_through: 2
    occupancy_threshold: 0.1
```

关键参数：

- `base_frame` 使用 TurtleBot3 已存在的 `base_footprint`。
- `min_laser_range` 和 `max_laser_range` 对应 Burger 仿真激光雷达范围。
- `resolution: 0.05` 表示地图每个栅格为 5 cm。
- `minimum_travel_distance` 和 `minimum_travel_heading` 控制何时向位姿图添加新节点。
- `do_loop_closing` 开启回环检测。

先使用这些基线参数。不要在首次运行前调节回环阈值。

### 4.3 创建启动文件

创建：

```text
src/turtlebot3/turtlebot3_navigation2/launch/slam_toolbox.launch.py
```

写入：

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory('turtlebot3_navigation2')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')

    default_params = os.path.join(
        navigation_share, 'param', 'slam_toolbox.yaml')
    slam_launch = os.path.join(
        slam_toolbox_share, 'launch', 'online_sync_launch.py')
    rviz_config = os.path.join(
        slam_toolbox_share, 'config', 'slam_toolbox_default.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz for mapping'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_params,
            description='Full path to Slam Toolbox parameters'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file,
            }.items()),
        Node(
            package='rviz2',
            executable='rviz2',
            name='slam_rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
```

ROS 2 launch 文件使用 Python 是 ROS 2 的常规做法；这里没有新增 Python 业务节点。

### 4.4 声明直接运行依赖

在 `turtlebot3_navigation2/package.xml` 中，将以下内容放到现有依赖区域：

```xml
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
```

其中 `nav2_bringup` 已经存在，只保留一行，不要重复添加。

### 4.5 构建和静态检查

```bash
cd ~/GitHub/tb3_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select turtlebot3_navigation2
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 slam_toolbox.launch.py --show-args
```

预期能够看到：

- `use_sim_time`
- `use_rviz`
- `slam_params_file`

### 4.6 使用同一个 rosbag 运行和验证 Slam Toolbox

停止 Gazebo、Cartographer、AMCL 和 Nav2。Slam Toolbox 必须读取阶段一使用的同一个 bag，并从头开始播放。

终端 1 启动 Slam Toolbox：

```bash
ros2 launch turtlebot3_navigation2 slam_toolbox.launch.py use_sim_time:=true
```

终端 2 播放与 Cartographer 完全相同的输入：

```bash
export SLAM_INPUT_BAG="$HOME/GitHub/tb3_jazzy_ws/bags/slam_input_20260714_152204"
ros2 bag play "$SLAM_INPUT_BAG" \
  --clock 50 \
  --rate 1.0 \
  --delay 2 \
  --topics /scan /odom /tf /tf_static
```

终端 3 在播放期间检查：

```bash
ros2 lifecycle get /slam_toolbox
ros2 topic hz /scan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

验收条件：

- `/slam_toolbox` 为 `active`。
- `/scan` 在回放期间保持稳定频率。
- `/map` 周期性更新。
- `map -> odom` 能持续查询，没有重复发布者警告。
- 输入 bag、播放速率、播放话题和 Cartographer 测试完全相同。

播放结束后保存地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "$HOME/GitHub/tb3_jazzy_ws/maps/replay/slam_toolbox_house"
```

将 Cartographer 和 Slam Toolbox 的输出地图路径、回环表现、重影和完整性填写到 [本次实验记录](./experiment-record-20260714-152204.md)。两种算法的差异现在只来自算法和参数，不来自机器人行驶路线。

建议提交第二个检查点：

```bash
git add src/turtlebot3/turtlebot3_navigation2/package.xml \
  src/turtlebot3/turtlebot3_navigation2/launch/slam_toolbox.launch.py \
  src/turtlebot3/turtlebot3_navigation2/param/slam_toolbox.yaml
git commit -m "Add Slam Toolbox mapping profile"
```

## 5. 阶段三：将 NavFn 替换为 SmacPlanner2D

NavFn 主要基于栅格势场生成路径。SmacPlanner2D 使用代价感知的 A* 搜索，可以通过 `cost_travel_multiplier` 更明确地避开高代价区域。

停止 Slam Toolbox 和遥控，保持 Gazebo 运行。编辑：

```text
src/turtlebot3/turtlebot3_navigation2/param/my_burger.yaml
```

找到 `planner_server`，将整个区块替换为：

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 10.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"
      tolerance: 0.15
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 2.0
      cost_travel_multiplier: 2.0
      use_final_approach_orientation: false
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
        do_refinement: true
        refinement_num: 2
```

参数含义：

- `tolerance: 0.15` 是规划器找不到精确终点时允许的近似路径范围，不等同于控制器的到点判定。
- `cost_travel_multiplier: 2.0` 让路径更偏向低代价区域，通常能减少贴墙。
- `downsample_costmap: false` 保留 5 cm 地图分辨率。
- `allow_unknown: true` 允许在包含未知区域的地图上规划；使用完整静态地图时影响较小。
- `smoother` 对 A* 路径进行平滑。

构建并检查：

```bash
colcon build --symlink-install --packages-select turtlebot3_navigation2
source install/setup.bash
ros2 launch turtlebot3_navigation2 my_nav2.launch.py --show-args
```

运行 Nav2 后查询实际插件：

```bash
ros2 param get /planner_server GridBased.plugin
```

预期：

```text
String value is: nav2_smac_planner::SmacPlanner2D
```

先使用原 DWB 控制器完成一次测试。这样可以把“全局规划器变化”和“局部控制器变化”分开验证。

验收条件：

- `planner_server` 成功进入 `active`。
- RViz 能生成全局路径。
- 路径不穿过障碍或膨胀区。
- 导航成功率不低于原 NavFn。

建议提交第三个检查点：

```bash
git add src/turtlebot3/turtlebot3_navigation2/param/my_burger.yaml
git commit -m "Use Smac 2D global planner"
```

## 6. 阶段四：将 DWB 替换为 MPPI

MPPI 会采样多组控制序列，在机器人运动模型上向前预测，再根据障碍、路径、目标和控制约束选择代价最低的速度命令。当前 Burger 是差速底盘，因此使用 `DiffDrive` 模型。

### 6.1 替换控制器配置

在 `my_burger.yaml` 中找到 `controller_server`，将整个区块替换为：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.1
      movement_time_allowance: 10.0
    goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.2
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56
      model_dt: 0.05
      batch_size: 2000
      ax_max: 3.0
      ax_min: -2.5
      ay_max: 0.0
      ay_min: 0.0
      az_max: 3.2
      vx_std: 0.2
      vy_std: 0.2
      wz_std: 0.4
      vx_max: 0.3
      vx_min: 0.0
      vy_max: 0.0
      wz_max: 1.0
      iteration_count: 1
      prune_distance: 1.7
      transform_tolerance: 0.1
      temperature: 0.3
      gamma: 0.015
      motion_model: "DiffDrive"
      visualize: true
      regenerate_noises: true
      TrajectoryVisualizer:
        trajectory_step: 5
        time_step: 3
      critics:
        - ConstraintCritic
        - CostCritic
        - GoalCritic
        - GoalAngleCritic
        - PathAlignCritic
        - PathFollowCritic
        - PathAngleCritic
        - PreferForwardCritic
      ConstraintCritic:
        enabled: true
        cost_power: 1
        cost_weight: 4.0
      GoalCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 1.4
      GoalAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.0
        threshold_to_consider: 0.5
      PreferForwardCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        threshold_to_consider: 0.5
      CostCritic:
        enabled: true
        cost_power: 1
        cost_weight: 3.81
        near_collision_cost: 253
        critical_cost: 300.0
        consider_footprint: false
        collision_cost: 1000000.0
        near_goal_distance: 1.0
        trajectory_point_step: 2
      PathAlignCritic:
        enabled: true
        cost_power: 1
        cost_weight: 14.0
        max_path_occupancy_ratio: 0.05
        trajectory_point_step: 4
        threshold_to_consider: 0.5
        offset_from_furthest: 20
        use_path_orientations: false
      PathFollowCritic:
        enabled: true
        cost_power: 1
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.4
      PathAngleCritic:
        enabled: true
        cost_power: 1
        cost_weight: 2.0
        offset_from_furthest: 4
        threshold_to_consider: 0.5
        max_angle_to_furthest: 1.0
        mode: 0
    enable_stamped_cmd_vel: true
```

这里有三个必须保持一致的约束：

1. `controller_frequency: 20.0` 的周期是 0.05 秒，与 `model_dt: 0.05` 对应。
2. `vx_max`、`wz_max`、`ax_max` 和 `az_max` 沿用当前 Burger 的速度与加速度范围。
3. `motion_model` 必须是 `DiffDrive`，Burger 不能使用横向速度。

主要 critic：

| critic | 作用 |
| --- | --- |
| `ConstraintCritic` | 淘汰违反运动学约束的轨迹 |
| `CostCritic` | 避免高代价区域和碰撞 |
| `GoalCritic` | 让机器人接近目标位置 |
| `GoalAngleCritic` | 让机器人接近目标方向 |
| `PathAlignCritic` | 让轨迹方向与全局路径对齐 |
| `PathFollowCritic` | 让预测轨迹跟随全局路径 |
| `PathAngleCritic` | 约束路径方向误差 |
| `PreferForwardCritic` | 优先前进，减少无意义倒车 |

### 6.2 构建和运行

```bash
colcon build --symlink-install --packages-select turtlebot3_navigation2
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

终端 1保持 Gazebo 运行，终端 2 启动 Nav2：

```bash
ros2 launch turtlebot3_navigation2 my_nav2.launch.py use_sim_time:=true
```

检查插件和生命周期：

```bash
ros2 param get /controller_server FollowPath.plugin
ros2 lifecycle get /controller_server
```

预期插件为：

```text
nav2_mppi_controller::MPPIController
```

控制器应为 `active`。RViz 中启用 MPPI 轨迹显示后，可以看到采样轨迹；如果界面明显卡顿，可以把 `visualize` 改为 `false`，这不影响控制算法。

建议提交第四个检查点：

```bash
git add src/turtlebot3/turtlebot3_navigation2/param/my_burger.yaml
git commit -m "Use MPPI path controller"
```

## 7. 阶段五：A/B 对比验证

### 7.1 启动基线配置

构建后，安装空间中会包含基线参数文件。使用现有 `params_file` 接口启动：

```bash
ros2 launch turtlebot3_navigation2 my_nav2.launch.py \
  use_sim_time:=true \
  params_file:="$(ros2 pkg prefix --share turtlebot3_navigation2)/param/my_burger_dwb_navfn.yaml"
```

### 7.2 启动升级配置

```bash
ros2 launch turtlebot3_navigation2 my_nav2.launch.py \
  use_sim_time:=true \
  params_file:="$(ros2 pkg prefix --share turtlebot3_navigation2)/param/my_burger.yaml"
```

不要同时启动两套 Nav2。每次切换配置前，完整停止上一套节点。

### 7.3 分别记录两套导航 rosbag

导航是闭环控制：控制器输出 `/cmd_vel` 后，Gazebo 中的机器人运动会产生新的里程计和激光数据。因此，不能把 DWB 运行产生的 bag 重放给 MPPI 并据此比较控制效果。两套配置必须分别在全新的 Gazebo 实例中执行相同目标序列。

两套配置还必须加载同一份 waypoint YAML，并选择同一种导航模式；否则每个点的到达判定和路径生成方式不同，指标不可直接比较。

每套配置启动后，先确认实际话题名称：

```bash
ros2 topic list --include-hidden-topics | sort
```

基线运行使用：

```bash
export PROFILE=navfn_dwb
export RUN_ID=01
```

升级配置运行使用：

```bash
export PROFILE=smac_mppi
export RUN_ID=01
```

每次测试选择其中一组变量，然后开始录制：

```bash
export BAG_ROOT="$HOME/bags/tb3_jazzy"
export NAV_BAG="$BAG_ROOT/nav_${PROFILE}_run_${RUN_ID}"
mkdir -p "$BAG_ROOT"

ros2 bag record \
  --use-sim-time \
  --storage mcap \
  --include-hidden-topics \
  --output "$NAV_BAG" \
  --topics \
  /clock /scan /odom /tf /tf_static /map /amcl_pose /cmd_vel \
  /plan /local_plan \
  /local_costmap/costmap /global_costmap/costmap \
  /follow_waypoints/_action/feedback \
  /follow_waypoints/_action/status \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status \
  /navigate_through_poses/_action/feedback \
  /navigate_through_poses/_action/status
```

依次发送 10 个固定目标。全部完成后在录包终端按 `Ctrl+C`，检查结果：

```bash
printf 'NAV_BAG=%s\n' "$NAV_BAG"
ros2 bag info --sort name "$NAV_BAG"
```

如果输出目录已经存在，rosbag 会拒绝覆盖。保留旧数据并递增 `RUN_ID`，不要删除先前实验。

如果列表中不存在某个非核心话题，例如 `/local_plan`，先从记录命令中删除该话题。`/clock`、`/scan`、`/odom`、`/tf`、`/map`、`/amcl_pose` 和 `/cmd_vel` 必须存在。

把两套 bag 的路径、Git commit、参数文件 SHA-256 和消息数填写到 [实验记录](./experiment-record.md)：

```bash
git rev-parse HEAD
sha256sum \
  src/turtlebot3/turtlebot3_navigation2/param/my_burger_dwb_navfn.yaml \
  src/turtlebot3/turtlebot3_navigation2/param/my_burger.yaml \
  src/turtlebot3/turtlebot3_navigation2/map/tb3_house.yaml \
  src/turtlebot3/turtlebot3_navigation2/map/tb3_house.pgm
```

### 7.4 回放导航 bag 进行分析

导航 bag 只用于复查 TF、定位、速度、路径和代价地图，不用于评价另一套控制器。

停止 Gazebo 和 Nav2。终端 1 启动 RViz：

```bash
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix --share turtlebot3_navigation2)/rviz/tb3_navigation2.rviz" \
  --ros-args -p use_sim_time:=true
```

终端 2 指定要分析的 bag，并以半速播放：

```bash
export NAV_BAG="$HOME/bags/tb3_jazzy/nav_smac_mppi_run_01"
ros2 bag info --sort name "$NAV_BAG"
ros2 bag play "$NAV_BAG" \
  --clock 50 \
  --rate 0.5 \
  --delay 2
```

播放时空格键暂停或继续，右方向键单步播放。降低 `--rate` 可以更清楚地观察路径重规划和障碍代价变化。

### 7.5 统一测试条件

两套配置必须保持以下条件一致：

- 同一个 `tb3_house` 地图。
- 同一个机器人出生点。
- 同一个 AMCL 初始位姿。
- 同样的 10 个目标和执行顺序。
- 每次测试前重启 Gazebo，避免残留状态。
- 不在一次对比中同时修改地图、速度、膨胀半径或目标容差。

使用下面的命令模板发送可复现目标：

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: X, y: Y, z: 0.0}, orientation: {z: QZ, w: QW}}}}" \
  --feedback
```

将 `X`、`Y`、`QZ`、`QW` 替换成记录好的目标值。

### 7.6 验收标准

| 指标 | 最低要求 |
| --- | --- |
| 导航成功率 | 至少 9/10，且不低于基线 |
| 碰撞次数 | 0 |
| 最终位置误差 | 不超过 0.15 m |
| 最终角度误差 | 不超过 0.2 rad |
| 路径合法性 | 不穿过障碍或膨胀区 |
| 稳定性 | 无持续原地振荡，无反复规划死循环 |

查询机器人最终位姿：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

位置误差计算：

```text
error_xy = sqrt((x_actual - x_goal)^2 + (y_actual - y_goal)^2)
```

对比表：

| 指标 | NavFn + DWB | Smac2D + MPPI |
| --- | --- | --- |
| 成功数 / 10 |  |  |
| 碰撞次数 |  |  |
| 恢复行为次数 |  |  |
| 振荡次数 |  |  |
| 平均到达时间 |  |  |
| 最大位置误差 |  |  |
| 最大角度误差 |  |  |

如果新方案没有优于基线，不要立即继续增加算法。先根据 rosbag 和日志判断问题来自地图、全局路径、局部代价地图还是控制器。

## 8. 调参顺序

始终一次只调整一组参数，并为每次实验记录 Git 提交或参数差异。

### 8.1 第一步：运动学限制

| 现象 | 优先检查 | 调整方向 |
| --- | --- | --- |
| 速度过快、转弯冲出路径 | `vx_max`、`wz_max` | 逐步降低 |
| 起步或停车突兀 | `ax_max`、`ax_min` | 降低绝对值 |
| 转向跟不上路径 | `wz_max`、`az_max` | 在底盘允许范围内提高 |

### 8.2 第二步：代价地图

| 现象 | 优先检查 | 调整方向 |
| --- | --- | --- |
| 路径贴墙 | `inflation_radius` | 增大 |
| 狭窄通道完全不可走 | `inflation_radius` | 减小，但不得小于安全余量 |
| 障碍残影 | clearing、raytrace 范围 | 检查传感器清除配置 |
| 动态障碍发现太慢 | `update_frequency` | 在性能允许时提高 |

不要为了通过狭窄通道把膨胀半径调到接近零。当前 `robot_radius: 0.1` 是碰撞边界，膨胀层还需要提供安全余量。

### 8.3 第三步：SmacPlanner2D

| 现象 | 参数 | 调整方向 |
| --- | --- | --- |
| 路径贴近高代价区域 | `cost_travel_multiplier` | 从 2.0 小幅提高 |
| 规划耗时过长 | `max_planning_time`、降采样 | 先测量，再考虑降采样 |
| 目标附近无法生成路径 | `tolerance` | 小幅提高，不能替代控制器到点容差 |

### 8.4 第四步：MPPI

| 现象 | 参数 | 调整方向 |
| --- | --- | --- |
| 路径跟随偏差大 | `PathFollowCritic.cost_weight` | 小幅提高 |
| 靠障碍过近 | `CostCritic.cost_weight` | 小幅提高 |
| 终点位置不稳定 | `GoalCritic.cost_weight` | 小幅提高 |
| 终点方向调整不足 | `GoalAngleCritic.cost_weight` | 小幅提高 |
| 轨迹抖动 | `gamma`、`temperature` | 每次只调整一个并复测 |
| 计算负载过高 | `batch_size`、`visualize` | 先关闭显示，再降低采样数 |

critic 权重每次调整不超过约 20%，完成相同 10 个目标后再决定是否保留。

### 8.5 最后调整到点容差

当前配置：

```yaml
xy_goal_tolerance: 0.15
yaw_goal_tolerance: 0.2
```

只有路径跟随和终点控制已经稳定时，才继续缩小容差。容差过小会导致机器人在目标附近频繁微调，降低整体成功率。

## 9. 常见问题

### 9.1 出现多个 `map -> odom` 发布者

原因：Cartographer、Slam Toolbox 或 AMCL 同时运行。

检查：

```bash
ros2 node list
ros2 topic info /tf --verbose
```

处理：

- 建图阶段只运行 Cartographer 或 Slam Toolbox 中的一个。
- 导航阶段停止 SLAM，只由 AMCL 定位。

### 9.2 Slam Toolbox 一直不是 active

```bash
ros2 lifecycle get /slam_toolbox
ros2 topic echo /clock --once
ros2 topic echo /scan --once
```

确认 `ros2 bag play` 正在使用 `--clock 50` 发布回放时钟，Slam Toolbox 使用 `use_sim_time:=true`，并且回放期间 `/scan` 有数据。此阶段不应启动 Gazebo。

### 9.3 没有地图或地图不更新

```bash
ros2 topic hz /map
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

如果缺少任意一段 TF，先修复 TF，不要通过提高回环阈值绕过问题。

### 9.4 Nav2 报插件加载失败

```bash
ros2 pkg prefix nav2_smac_planner
ros2 pkg prefix nav2_mppi_controller
```

确认 YAML 中使用 Jazzy 插件名称：

```text
nav2_smac_planner::SmacPlanner2D
nav2_mppi_controller::MPPIController
```

修改配置后必须重新构建并重新加载 `install/setup.bash`。

### 9.5 MPPI 报控制周期不匹配

确认：

```yaml
controller_frequency: 20.0
model_dt: 0.05
```

20 Hz 的周期正好是 0.05 秒。不要只修改其中一个参数。

### 9.6 机器人在终点附近振荡

按以下顺序检查：

1. 确认 AMCL 位姿没有明显跳变。
2. 恢复 `xy_goal_tolerance: 0.15` 和 `yaw_goal_tolerance: 0.2`。
3. 检查目标是否位于障碍膨胀区。
4. 检查 `GoalCritic` 和 `GoalAngleCritic`，不要同时大幅提高所有权重。

### 9.7 rosbag 录制或回放失败

先确认目录和元数据：

```bash
test -d "$NAV_BAG"
ros2 bag info --sort name "$NAV_BAG"
```

常见原因：

- 输出目录已存在：递增 `RUN_ID`，不要覆盖旧实验。
- bag 没有消息：录制时没有收到 `/clock`，或者 `--use-sim-time` 后 Gazebo 尚未开始运行。
- 回放后节点无响应：确认算法节点设置 `use_sim_time:=true`，并使用 `--clock 50`。
- TF 冲突：SLAM 原始输入 bag 录制时错误地启动了 SLAM 或 AMCL，应重新录制原始输入。
- 部分话题缺失：先执行 `ros2 topic list --include-hidden-topics`，只删除不存在的非核心话题。

### 9.8 恢复某个阶段

查看学习分支提交：

```bash
git log --oneline --decorate -10
```

如果只是查看旧配置：

```bash
git show COMMIT:src/turtlebot3/turtlebot3_navigation2/param/my_burger.yaml
```

不要使用 `git reset --hard`。需要撤销已提交阶段时使用：

```bash
git revert COMMIT
```

## 10. 完成条件

满足以下条件后，二维系统升级完成：

- Slam Toolbox 能独立完成 `turtlebot3_house` 建图和地图保存。
- Cartographer 仍可作为基准启动，不存在配置覆盖。
- SmacPlanner2D 和 MPPI 节点都能进入 `active`。
- 相同 10 个目标至少成功 9 个。
- 没有碰撞和持续振荡。
- 到点误差满足 `0.15 m` 和 `0.2 rad`。
- rosbag 和实验表能够说明新方案是否真正优于基线。
- [实验记录](./experiment-record.md) 已填写环境、bag、目标、结果和异常信息。

最后执行完整构建：

```bash
cd ~/GitHub/tb3_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## 11. 后续学习路线

建议按以下顺序继续：

1. 为 Gazebo Burger 增加 RGB-D 深度相机。
2. 将深度点云作为 `PointCloud2` 接入 Nav2 `VoxelLayer`，但仍使用二维地图。
3. 使用 Nav2 Waypoint Follower 实现固定巡逻。
4. 增加前沿检测，实现自主探索和自动建图。
5. 学习 `nav2_core::GlobalPlanner` 或 `nav2_core::Controller`，使用 C++ 和 pluginlib 编写自定义插件。
6. 只有明确需要三维地图或视觉定位时，再引入 RTAB-Map。

## 12. 官方资料

- [Slam Toolbox Jazzy 文档](https://docs.ros.org/en/ros2_packages/jazzy/api/slam_toolbox/)
- [Nav2 MPPI Controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html)
- [Nav2 SmacPlanner2D](https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html)
- [Nav2 Planner 插件教程](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html)
- [Nav2 Controller 插件教程](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html)
