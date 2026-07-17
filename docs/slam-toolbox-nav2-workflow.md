# Slam Toolbox 建图到 Nav2 优化全流程

本文是一份可直接执行的 Runbook，目标环境为 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Sim、TurtleBot3 Burger 和二维激光雷达。它使用 Gazebo 实时传感器数据完成 Slam Toolbox 在线建图，再使用静态地图、AMCL 和 Nav2 进行分阶段导航优化。

详细原理、完整 SmacPlanner2D/MPPI 参数和 Cartographer 对照实验参见 [2D SLAM 与 Nav2 优化学习指南](./2d-slam-nav2-learning-guide.md)。每轮结果填写到 [实验记录模板](./experiment-record.md)。

## 1. 全流程与约束

```text
Gazebo 实时仿真
  -> Slam Toolbox 在线建图
  -> 键盘或手柄遥控探索
  -> 保存 PGM/YAML 和 pose graph
  -> 地图质量验收
  -> 关闭 Slam Toolbox
  -> 静态地图 + AMCL + Nav2
  -> NavFn + DWB 基线
  -> SmacPlanner2D + DWB
  -> SmacPlanner2D + MPPI
  -> 固定 waypoint 闭环复测和 rosbag 对比
```

整个流程遵守以下约束：

- 每次重新测试 Slam Toolbox 参数都重启 Gazebo，并保持出生点、探索路线、速度、探索时长和结束位置一致。
- 导航实验使用同一张地图、同一出生点、同一 AMCL 初始位姿和同一组 waypoint。
- 一轮实验只调整一组参数，不同时修改 SLAM、定位、代价地图、规划器和控制器。
- 每个 Nav2 配置都必须在新启动的 Gazebo 中独立运行并重新录包。
- 正式静态地图导航时停止 Slam Toolbox，避免多个节点同时发布 `map -> odom`。

在线手工探索的每次输入不会完全一致。比较不同 Slam Toolbox 参数时，必须记录实际路线、用时、参数文件 SHA-256 和输出地图 SHA-256，并把结论限定在本轮在线实验条件内。

## 2. 环境和统一变量

每个终端先执行：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
cd "$TB3_WS"

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

本次实验使用以下固定路径：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export SLAM_MAP_PREFIX="$TB3_WS/maps/online/slam_toolbox_house"
export WAYPOINT_FILE="$TB3_WS/bags/slam_input_20260714_152204/waypoint2.yaml"
export NAV_BAG_ROOT="$TB3_WS/bags/navigation"
```

检查依赖：

```bash
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_bringup
ros2 pkg prefix turtlebot3_navigation2
```

构建当前包并检查启动接口：

```bash
colcon build --symlink-install --packages-select turtlebot3_navigation2
source install/setup.bash

ros2 launch turtlebot3_navigation2 slam_toolbox.launch.py --show-args
ros2 launch turtlebot3_navigation2 my_nav2.launch.py --show-args
```

## 3. 使用 Slam Toolbox 在线建图

在线建图直接使用 Gazebo 发布的 `/clock`、`/scan`、`/odom`、`/tf` 和 `/tf_static`，不录制或回放建图 rosbag。

### 3.1 启动 Gazebo

终端 1：

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

等待机器人生成并确认仿真没有暂停：

```bash
ros2 topic echo /clock --once
ros2 topic list | sort
```

至少应存在 `/clock`、`/scan`、`/odom`、`/tf` 和 `/tf_static`。

### 3.2 启动 Slam Toolbox 和 RViz

终端 2：

```bash
ros2 launch turtlebot3_navigation2 slam_toolbox.launch.py \
  use_sim_time:=true \
  use_rviz:=true
```

当前项目使用同步节点和以下配置：

```text
src/turtlebot3/turtlebot3_navigation2/param/slam_toolbox.yaml
```

终端 3：

```bash
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix --share turtlebot3_navigation2)/rviz/slam_toolbox.rviz" \
  --ros-args -p use_sim_time:=true
```

RViz 的 Fixed Frame 应为 `map`，并至少启用 `Map`、`LaserScan`、`TF` 和 `RobotModel`。如果左侧 Displays 只有 Grid，点击 `Add -> By topic -> /map -> Map`；SlamToolboxPlugin 只是控制面板，不负责渲染地图。

### 3.3 启动遥控并探索

终端 4，键盘和手柄二选一：

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

或：

```bash
ros2 launch turtlebot3_teleop xbox_teleop.launch.py
```

沿固定路线低速探索：

- 先扫描出生点周围并确认初始墙体方向正确。
- 覆盖所有房间、墙角、桌脚和障碍边缘。
- 进入窄通道时保持低速，避免快速转向造成扫描匹配误差。
- 对置信度低的区域从不同方向重复扫描。
- 最后回到起点附近，给回环检测足够时间完成优化。
- 不要启动 Cartographer、AMCL 或 Nav2，避免多个节点发布地图和 `map -> odom`。

### 3.4 运行状态检查

开始移动前执行：

```bash
ros2 lifecycle get /slam_toolbox
ros2 param get /slam_toolbox use_sim_time
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint --once
```

移动约 5 秒后执行：

ros2 topic info /map -v
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map base_footprint --once
```

预期结果：

- `/slam_toolbox` 为 `active`。
- `use_sim_time` 为 `true`。
- `/map` 有发布者，`width` 和 `height` 大于零。
- `map -> odom -> base_footprint` TF 可查询。
- RViz 中激光数据和墙体基本重合。

以下日志不是建图失败：

```text
clipped range threshold to be within minimum and maximum range
Specified options.num_threads: 50 exceeds maximum ... Bounding to ... 16
```

前者表示激光范围被限制到传感器有效范围；后者表示 Ceres 自动把线程数限制为编译时允许的最大值。

## 4. 保存和验收地图

### 4.1 保存二维栅格地图

保持 Slam Toolbox 和 `/map` 正常发布：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export SLAM_MAP_PREFIX="$TB3_WS/maps/online/slam_toolbox_house"

mkdir -p "$(dirname "$SLAM_MAP_PREFIX")"

ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f "$SLAM_MAP_PREFIX" \
  --free 0.196 \
  --occ 0.65 \
  --fmt pgm \
  --mode trinary
```

输出：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export SLAM_MAP_PREFIX="$TB3_WS/maps/online/slam_toolbox_house"

printf '%s\n' "$SLAM_MAP_PREFIX.yaml" "$SLAM_MAP_PREFIX.pgm"
```

`free_thresh` 固定为 `0.196`。灰度 205 对应约 `0.1961` 的占用概率，使用 `0.25` 可能把未知区域加载成自由区域。

### 4.2 序列化 pose graph

如果以后需要继续建图或使用 Slam Toolbox 定位模式，同时保存 pose graph：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export SLAM_MAP_PREFIX="$TB3_WS/maps/online/slam_toolbox_house"

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$SLAM_MAP_PREFIX'}"
```

响应中的 `result: 0` 表示成功。PGM/YAML 用于 Nav2 静态地图导航，pose graph 用于 Slam Toolbox 续建或定位，两者用途不同。

### 4.3 文件和地图质量检查

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export SLAM_MAP_PREFIX="$TB3_WS/maps/online/slam_toolbox_house"

sed -n '1,20p' "$SLAM_MAP_PREFIX.yaml"
file "$SLAM_MAP_PREFIX.pgm"
sha256sum "$SLAM_MAP_PREFIX.yaml" "$SLAM_MAP_PREFIX.pgm"
```

地图必须满足：

- 分辨率为 `0.05` m。
- YAML 中 `free_thresh: 0.196`、`occupied_thresh: 0.65`。
- 外墙、房间和走廊闭合，没有明显断裂。
- 回到起点后没有明显双墙、重影或整体跳变。
- 桌脚等障碍没有大面积放射状噪声。
- 窄通道的自由宽度大于机器人直径与安全余量。
- 未探索区域保持未知，不作为自由空间参与导航。

如果地图本身存在双墙或墙体偏移，先调整 Slam Toolbox 或重新采集输入；不要用减小 Nav2 膨胀半径掩盖地图错误。

## 5. 使用 Slam Toolbox 地图启动 Nav2

完成地图保存后停止 rosbag player、Slam Toolbox 和建图 RViz。导航阶段只保留 Gazebo、map_server、AMCL 和 Nav2。

### 5.1 启动全新的 Gazebo

终端 1：

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

每轮 A/B 测试都重新启动 Gazebo，避免残留速度、TF、代价地图和生命周期状态。

### 5.2 启动静态地图导航

终端 2：

```bash
ros2 launch turtlebot3_navigation2 my_nav2.launch.py use_sim_time:=true  map:=/home/yzh/GitHub/tb3_jazzy_ws/maps/online/slam_toolbox_house.yaml initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

初始位姿是机器人在 `map` 坐标系中的位姿，不一定等于 Gazebo world 中的出生坐标。如果地图坐标系或出生点改变，应在 RViz 用 `2D Pose Estimate` 校准后，再把相同值固定到后续所有实验。

检查定位：

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_footprint --once
```

发送目标前观察至少 10 秒：

- `/scan` 与静态地图墙体持续重合。
- `map -> odom` 不连续跳变。
- 小车静止时 AMCL 位姿没有明显漂移。
- 局部和全局代价地图不存在整片异常障碍。

## 6. 固定 waypoint 和导航录包

### 6.1 目标执行方式

确认 waypoint 文件存在，然后在 RViz 加载：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export WAYPOINT_FILE="$TB3_WS/bags/slam_input_20260714_152204/waypoint2.yaml"

printf 'WAYPOINT_FILE=%s\n' "$WAYPOINT_FILE"
test -f "$WAYPOINT_FILE"
```

使用 `Start Waypoint Following`。该模式为每个 waypoint 执行独立的 `NavigateToPose`，适合记录每个点的耗时、剩余距离、恢复次数和最终误差；闭环路线的终点靠近起点时，也不会因为最终目标已经位于容差内而整条任务立即成功。

所有配置必须使用同一份 waypoint 文件和相同执行顺序。

### 6.2 录制导航过程

发送 waypoint 前，在独立终端设置配置名称：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export NAV_BAG_ROOT="$TB3_WS/bags/navigation"
export PROFILE=slamtoolbox_navfn_dwb
export RUN_ID=01
mkdir -p "$NAV_BAG_ROOT"
export NAV_BAG="$NAV_BAG_ROOT/nav_${PROFILE}_run_${RUN_ID}"
```

录制：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export NAV_BAG_ROOT="$TB3_WS/bags/navigation"
export PROFILE="${PROFILE:-slamtoolbox_navfn_dwb}"
export RUN_ID="${RUN_ID:-01}"
export NAV_BAG="$NAV_BAG_ROOT/nav_${PROFILE}_run_${RUN_ID}"
mkdir -p "$NAV_BAG_ROOT"

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

完成全部 waypoint 后按 `Ctrl+C`，然后检查：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export SLAM_MAP_PREFIX="$TB3_WS/maps/online/slam_toolbox_house"
export NAV_BAG_ROOT="$TB3_WS/bags/navigation"
export PROFILE="${PROFILE:-slamtoolbox_navfn_dwb}"
export RUN_ID="${RUN_ID:-01}"
export NAV_BAG="$NAV_BAG_ROOT/nav_${PROFILE}_run_${RUN_ID}"

printf 'NAV_BAG=%s\n' "$NAV_BAG"
ros2 bag info --sort name "$NAV_BAG"

git rev-parse HEAD
sha256sum \
  "$TB3_WS/src/turtlebot3/turtlebot3_navigation2/param/my_burger.yaml" \
  "$SLAM_MAP_PREFIX.yaml" \
  "$SLAM_MAP_PREFIX.pgm"
```

如果某个可选话题不存在，先从命令中删除该话题；`/clock`、`/scan`、`/odom`、`/tf`、`/map`、`/amcl_pose`、`/cmd_vel` 和两个 `NavigateToPose` action 反馈话题是核心数据。

## 7. 分阶段优化 Nav2

调参前先记录当前参数文件 SHA-256。每个阶段至少完整执行一次相同 waypoint；阶段不通过时回退，不继续叠加下一项修改。

### 7.1 阶段 A：NavFn + DWB 基线

当前 `my_burger.yaml` 使用：

```text
nav2_navfn_planner::NavfnPlanner
dwb_core::DWBLocalPlanner
```

运行时确认：

```bash
ros2 param get /planner_server GridBased.plugin
ros2 param get /controller_server FollowPath.plugin
```

使用 `PROFILE=slamtoolbox_navfn_dwb` 完成一轮测试。这一轮是后续所有优化的比较基线。

### 7.2 阶段 B：先稳定定位和代价地图

仅在 `/scan` 与地图稳定重合后调整导航参数，顺序如下：

1. AMCL 的 `update_min_d`、`update_min_a`、粒子数量和激光模型。
2. `robot_radius` 或 footprint，必须反映 Burger 实际外形。
3. 激光障碍层的 `obstacle_max_range` 和 `raytrace_max_range`。
4. `inflation_radius` 和 `cost_scaling_factor`。

当前起点参数为：

```text
AMCL update_min_d: 0.05 m
AMCL update_min_a: 0.05 rad
robot_radius: 0.10 m
inflation_radius: 0.25 m
cost_scaling_factor: 5.0
```

窄通道调参原则：

- 先确认静态地图墙体没有增厚或重影。
- 再确认实时 `/scan` 没有因为定位跳变反复覆盖路径。
- 最后才减小膨胀半径，每次只减小 `0.02` 到 `0.05` m。
- 不要降低占用阈值或关闭障碍层来强行穿过未知区域。

每次改动后重新启动 Nav2，使用原 NavFn+DWB 完成同一轮 waypoint。定位或代价地图不稳定时，不进入下一阶段。

### 7.3 阶段 C：SmacPlanner2D + DWB

按照 [学习指南第 5 节](./2d-slam-nav2-learning-guide.md) 只替换 `planner_server`，控制器继续使用 DWB。核心配置是：

```yaml
GridBased:
  plugin: "nav2_smac_planner::SmacPlanner2D"
  tolerance: 0.15
  downsample_costmap: false
  allow_unknown: true
  max_planning_time: 2.0
  cost_travel_multiplier: 2.0
```

构建并确认：

```bash
colcon build --symlink-install --packages-select turtlebot3_navigation2
source install/setup.bash
ros2 param get /planner_server GridBased.plugin
ros2 param get /controller_server FollowPath.plugin
```

预期分别为 `nav2_smac_planner::SmacPlanner2D` 和 `dwb_core::DWBLocalPlanner`。设置：

```bash
export PROFILE=slamtoolbox_smac_dwb
```

重新启动 Gazebo 和 Nav2，完成同一轮 waypoint。只有路径不穿障碍、成功率不下降并且窄通道表现改善时，才保留 SmacPlanner2D。

### 7.4 阶段 D：SmacPlanner2D + MPPI

按照 [学习指南第 6 节](./2d-slam-nav2-learning-guide.md) 只替换 `controller_server`。Burger 使用差速模型，必须保持：

```yaml
controller_frequency: 20.0
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "DiffDrive"
  model_dt: 0.05
  vx_max: 0.3
  vx_min: 0.0
  vy_max: 0.0
  wz_max: 1.0
```

`controller_frequency: 20.0` 对应 0.05 秒周期，必须与 `model_dt: 0.05` 一致。

构建并确认：

```bash
colcon build --symlink-install --packages-select turtlebot3_navigation2
source install/setup.bash
ros2 param get /planner_server GridBased.plugin
ros2 param get /controller_server FollowPath.plugin
```

设置：

```bash
export PROFILE=slamtoolbox_smac_mppi
```

重新启动全部导航进程并完成同一轮 waypoint。MPPI 调参顺序为速度和加速度限制、预测时域、障碍 critic、路径 critic、目标 critic；一次只调整一个 critic 或一组运动学参数。

### 7.5 阶段 E：最后调整到点和行为树

只有规划和控制稳定后才调整：

```text
SimpleGoalChecker.xy_goal_tolerance
SimpleGoalChecker.yaw_goal_tolerance
SimpleProgressChecker.required_movement_radius
SimpleProgressChecker.movement_time_allowance
BT 重规划频率和 RemovePassedGoals 半径
```

注意区分：

- 规划器 `tolerance`：找不到精确终点时允许的近似规划范围。
- goal checker 容差：Nav2 判断任务成功的最终位置和角度误差。
- DWB 内部 `xy_goal_tolerance`：控制器轨迹评分使用，不代替 goal checker。

容差过大会提前报告到达；容差过小会导致终点附近反复修正。修改后必须重新执行全部 waypoint，不能只测试一个终点。

## 8. 指标和验收标准

### 8.1 每个 waypoint 记录

| 指标 | 数据来源 | 定义 |
| --- | --- | --- |
| 是否成功 | action status | `NavigateToPose` 最终为 `SUCCEEDED` |
| 到达时间 | action feedback | 最后一条 feedback 的 `navigation_time` |
| 终点位置残差 | waypoint + `current_pose` | `sqrt((x-x_goal)^2 + (y-y_goal)^2)` |
| 终点角度残差 | waypoint + `current_pose` | 归一化到 `[-pi, pi]` 的 yaw 差绝对值 |
| 恢复次数 | action feedback | 最后一条或最大 `number_of_recoveries` |
| 路径长度 | `/odom` 或 `/plan` | 连续位置点之间距离之和 |

位置和角度残差是 Nav2 估计位姿相对目标的残差，不是真实物理误差。要评价真实误差，需要额外录制 Gazebo ground truth。

### 8.2 整轮运行记录

- waypoint 成功率。
- 成功目标的平均到达时间。
- 最大位置残差和对应 waypoint。
- 最大角度残差和对应 waypoint。
- 总恢复次数。
- 窄通道 waypoint7 附近是否停滞或恢复。
- 全局路径是否重复、折返或穿过高代价区域。
- AMCL 与 `map -> odom` 是否出现明显跳变。

“明显振荡”统一定义为：机器人低速或近乎原地时，在 5 秒内角速度符号反转至少 3 次，同时到目标距离没有持续下降。终点附近一次或两次方向修正不计为持续振荡。

### 8.3 阶段验收

每个升级阶段至少满足：

- 10 个 waypoint 全部完成，或成功率不低于上一阶段。
- 没有碰撞和持续振荡。
- waypoint7 窄通道不出现长时间原地恢复。
- 平均到达时间没有无解释的大幅上升。
- 最大位置和角度残差不超过 goal checker 容差的合理范围。
- `/scan` 与地图全程保持一致，定位跳变没有增多。

如果缺少碰撞接触话题或 ground truth，只能记录“未采集，无法判定”，不能填写为零。

## 9. 回放导航 bag

导航 bag 用于复查已发生的路径、定位和控制行为，不能作为另一控制器的闭环输入。

停止 Gazebo 和 Nav2。终端 1：

```bash
ros2 run rviz2 rviz2 \
  -d "$(ros2 pkg prefix --share turtlebot3_navigation2)/rviz/tb3_navigation2.rviz" \
  --ros-args -p use_sim_time:=true
```

终端 2：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export NAV_BAG_ROOT="$TB3_WS/bags/navigation"
export PROFILE="${PROFILE:-slamtoolbox_navfn_dwb}"
export RUN_ID="${RUN_ID:-01}"
export NAV_BAG="$NAV_BAG_ROOT/nav_${PROFILE}_run_${RUN_ID}"

ros2 bag info --sort name "$NAV_BAG"
ros2 bag play "$NAV_BAG" \
  --clock 50 \
  --rate 0.5 \
  --delay 2
```

使用空格暂停、右方向键单步播放，重点观察 `/plan`、`/local_plan`、两层 costmap、`/scan` 和 `map -> odom` 的变化时序。

## 10. 常见问题

### 10.1 ROS Time 增长但 RViz 没有地图

先检查 RViz 左侧是否存在 `Map` 显示项，并把 Topic 设置为 `/map`。然后执行：

```bash
ros2 lifecycle get /slam_toolbox
ros2 topic info /map
ros2 topic echo /map --once --field info
```

### 10.2 Slam Toolbox 收不到激光或 TF

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint --once
ros2 run tf2_ros tf2_echo base_footprint base_scan --once
```

确认 Gazebo 正在发布 `/scan /odom /tf /tf_static`，Gazebo 没有暂停，并且所有进程都使用 `use_sim_time:=true`。

### 10.3 地图长时间不更新

当前 `map_update_interval` 为 5 秒，`minimum_travel_distance` 和 `minimum_travel_heading` 都是 0.5。确认机器人实际移动了足够距离或角度，不要仅根据启动后的第一秒判断失败。

### 10.4 多个 `map -> odom` 发布者

```bash
ros2 node list
ros2 topic info /map -v
```

建图时只运行 Slam Toolbox；静态地图导航时停止 Slam Toolbox，只运行 AMCL。

### 10.5 窄通道被代价地图封死

按以下顺序排查：

1. 静态地图是否双墙、墙体过厚或未知区侵入通道。
2. `/scan` 是否与地图错位并随 AMCL 调整反复移动。
3. 机器人半径或 footprint 是否过大。
4. 障碍层是否把无效激光标成障碍。
5. 最后才调整 `inflation_radius` 和 `cost_scaling_factor`。

### 10.6 小车未到目标就拐弯或折返

检查局部路径是否提前朝下一个 waypoint 转向、全局路径是否重规划成重复段，以及 goal checker 与控制器内部容差是否不一致。Waypoint Following 每个点独立执行，不应通过扩大容差掩盖路径重复。

### 10.7 rosbag 输出目录已存在

不要删除旧实验；递增 `RUN_ID`：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export NAV_BAG_ROOT="$TB3_WS/bags/navigation"
export PROFILE="${PROFILE:-slamtoolbox_navfn_dwb}"
export RUN_ID=02
export NAV_BAG="$NAV_BAG_ROOT/nav_${PROFILE}_run_${RUN_ID}"
```

## 11. 每轮实验完成检查

- [ ] Gazebo world、机器人出生点、探索路线和探索用时已记录。
- [ ] Slam Toolbox 参数文件 SHA-256 已记录。
- [ ] 地图 YAML、PGM 和 pose graph 已保存。
- [ ] 地图分辨率和阈值已经检查。
- [ ] 固定地图、初始位姿和 waypoint 没有变化。
- [ ] Nav2 实际 planner/controller 插件已经查询。
- [ ] 导航 bag 包含核心话题和 action feedback。
- [ ] 每个 waypoint 的成功、耗时、残差和恢复次数已填写。
- [ ] 异常现象包含时间戳和复现步骤。
- [ ] 下一阶段只修改一组参数。

完成所有阶段后，把结果填写到 [实验记录](./experiment-record.md)，并保留对应 Git commit、参数校验值、地图校验值和 bag 路径。
