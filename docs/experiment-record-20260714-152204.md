# TurtleBot3 实验记录：slam_input_20260714_152204

本文对应原始输入 bag `bags/slam_input_20260714_152204`，基于 [实验记录模板](./experiment-record.md) 创建，并按照 [Slam Toolbox 建图到 Nav2 优化全流程](./slam-toolbox-nav2-workflow.md) 与 [2D SLAM 与 Nav2 优化学习指南](./2d-slam-nav2-learning-guide.md) 继续填写后续 SLAM 和 Nav2 结果。

Bag 文件保存在工作区 `bags/`，报告只记录路径和校验值，不将 MCAP 文件提交到 Git。

## 1. 实验标识

| 项目 | 记录 |
| --- | --- |
| 实验名称 | TurtleBot3 House 原始 SLAM 输入录制 |
| 实验日期与时区 | 2026-07-14 15:22:04，Asia/Shanghai（由 bag 名称记录） |
| 操作者 | 已脱敏 |
| 实验目的 | 为 Cartographer 与 Slam Toolbox 提供相同的 `/scan`、里程计和 TF 输入 |
| Git 分支 | `feature/slam-nav2-upgrade` |
| Git commit | `eeaa8067cb43055d482e96088fd3ae21f35d35a6` |
| 工作区是否干净 | 否；生成报告时 `.gitignore` 有未提交修改，录制时状态未核实 |

采集基础信息：

```bash
date --iso-8601=seconds
git branch --show-current
git rev-parse HEAD
git status --short
```

## 2. 软件与仿真环境

| 项目 | 记录 |
| --- | --- |
| 操作系统 | Ubuntu 24.04.4 LTS (Noble Numbat) |
| ROS 发行版 | Jazzy |
| RMW 实现 | `rmw_fastrtps_cpp` |
| Gazebo 版本 | 8.11.0 |
| `turtlebot3` 包版本 | 2.3.7 |
| `slam_toolbox` 包版本 | 2.8.5 (`ros-jazzy-slam-toolbox`) |
| `navigation2` 包版本 | 1.3.12 (`ros-jazzy-navigation2`) |
| `TURTLEBOT3_MODEL` | burger |
| Gazebo world | `turtlebot3_house` |
| CPU | AMD Ryzen 7 5800H with Radeon Graphics |
| 内存 | 15 GiB |

采集命令：

```bash
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-default}"
printf 'TURTLEBOT3_MODEL=%s\n' "$TURTLEBOT3_MODEL"
gz sim --versions
ros2 pkg prefix turtlebot3_navigation2
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_bringup
```

## 3. 配置与地图校验值

同一轮 A/B 实验只能修改目标算法对应的配置。地图、机器人模型、初始位姿和目标序列必须保持一致。

| 文件 | SHA-256 |
| --- | --- |
| `my_burger_dwb_navfn.yaml` | 未创建 |
| `my_burger.yaml` | `c664c342ee14d8b8c94fde5c1801a5ee66d00ceaaf72fa261afffb8a60c10220` |
| `slam_toolbox.yaml` | 未创建 |
| `tb3_house.yaml` | `6f5c6e644f0da2175f8a1606e9ddca3b9f9f6517330826e71fe17cd80bafce46` |
| `tb3_house.pgm` | `da790a26b299e32b08db4c2c21f8736be46986a6433fd71a1e34379569266132` |

采集命令：

```bash
sha256sum \
  src/turtlebot3/turtlebot3_navigation2/param/my_burger_dwb_navfn.yaml \
  src/turtlebot3/turtlebot3_navigation2/param/my_burger.yaml \
  src/turtlebot3/turtlebot3_navigation2/param/slam_toolbox.yaml \
  src/turtlebot3/turtlebot3_navigation2/map/tb3_house.yaml \
  src/turtlebot3/turtlebot3_navigation2/map/tb3_house.pgm
```

如果某个文件尚未创建，在记录中标记“未创建”，不要伪造校验值。

## 4. 统一 SLAM 输入 bag

### 4.1 Bag 元数据

| 项目 | 记录 |
| --- | --- |
| Bag 名称 | `slam_input_20260714_152204` |
| 完整路径 | `$TB3_WS/bags/slam_input_20260714_152204` |
| 存储格式 | MCAP |
| ROS 发行版 | Jazzy |
| 文件大小 | 127.1 MiB（`du` 显示 128M） |
| 开始时间 | 仿真时间 31.187 s；rosbag 显示 1970-01-01 08:00:31.187 |
| 结束时间 | 仿真时间 453.457 s；rosbag 显示 1970-01-01 08:07:33.457 |
| 持续时间 | 422.270 s（约 7 分 2.27 秒） |
| 总消息数 | 905995 |
| `/scan` 消息数 | 2112 |
| `/odom` 消息数 | 21113 |
| `/tf` 消息数 | 29559 |
| `/tf_static` 消息数 | 1 |
| 录制时是否未启动 SLAM/Nav2 | 是 |
| 探索路线说明 | 整体房间扫描 |

采集命令：

```bash
export TB3_WS="${TB3_WS:-$HOME/GitHub/tb3_jazzy_ws}"
export SLAM_INPUT_BAG="$TB3_WS/bags/slam_input_20260714_152204"
du -sh "$SLAM_INPUT_BAG"
ros2 bag info --sort name "$SLAM_INPUT_BAG"
find "$SLAM_INPUT_BAG" -maxdepth 1 -type f -print0 \
  | sort -z \
  | xargs -0 sha256sum
```

Bag 文件校验值：

```text
6f0500b59b162431aaea9e612a9e9ab4adc910a318fe95a2597891207225f6e6  metadata.yaml
a2fbbef78698582e4aff05424281ea2072541712e6d51e67be1758d4cd341735  slam_input_20260714_152204_0.mcap
```

完整话题统计：

| 话题 | 类型 | 消息数 |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | 422272 |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | 8671 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 422267 |
| `/odom` | `nav_msgs/msg/Odometry` | 21113 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2112 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 29559 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 |

### 4.2 输入完整性检查

- [x] 包含 `/clock`。
- [x] 包含 `/scan` 且消息数大于零。
- [x] 包含 `/odom` 且消息数大于零。
- [x] 包含 `/tf` 和 `/tf_static`。
- [x] 不包含 `/map`。
- [x] 录制时没有 Cartographer、Slam Toolbox、AMCL 或 Nav2 节点。
- [x] 机器人完成预定路线并回到起点附近。

输入异常与处理：

```text
无明显异常
```

## 5. SLAM A/B 结果

两种算法必须使用第 4 节中的同一个 bag，并使用相同的 `--clock 50 --rate 1.0 --delay 2` 播放参数。

### 5.1 Cartographer

| 项目 | 记录 |
| --- | --- |
| 输入 bag | `$TB3_WS/bags/slam_input_20260714_152204` |
| 配置文件 | `turtlebot3_lds_2d.lua` |
| 输出地图 YAML | `$TB3_WS/maps/replay/cartographer_house.yaml` |
| 输出地图图像 | `$TB3_WS/maps/replay/cartographer_house.pgm` |
| 是否成功回环 | 是 |
| 是否出现明显双墙 | 否 |
| 是否存在地图跳变 | 否 |
| 房间与走廊是否闭合 | 否 |
| 错误或警告 | 无 |

观察记录：

```text
SLAM在桌脚周边位置需要反复扫描，否则地图置信度不高
```

### 5.2 Slam Toolbox

| 项目 | 记录 |
| --- | --- |
| 输入 bag |  |
| 配置文件 | `slam_toolbox.yaml` |
| 输出地图 YAML |  |
| 输出地图图像 |  |
| 地图校验值 |  |
| 是否成功回环 | 是 / 否 |
| 是否出现明显双墙 | 是 / 否 |
| 是否存在地图跳变 | 是 / 否 |
| 房间与走廊是否闭合 | 是 / 否 |
| 错误或警告 |  |

观察记录：

```text
在此记录回环位置、重影区域、地图缺失和运行时异常
```

### 5.3 SLAM 对比结论

| 指标 | Cartographer | Slam Toolbox | 更优者 |
| --- | --- | --- | --- |
| 地图完整性 |  |  |  |
| 墙体重影 |  |  |  |
| 回环稳定性 |  |  |  |
| 地图跳变 |  |  |  |
| 运行错误 |  |  |  |

结论及依据：

```text
在此说明哪种算法更适合当前场景，并引用具体 bag、地图和观察结果
```

## 6. 固定导航条件

### 6.1 地图和初始位姿

| 项目 | 记录 |
| --- | --- |
| 地图路径 | `maps/replay/cartographer_house.yaml` |
| 地图 YAML SHA-256 | `7a7ca6b2e9f43bf5fa1901c1f5ea890e8e5db54caeb8e8f998e2493e491961cc` |
| 地图 PGM SHA-256 | `4cb4637bf79986dd23174b61a4714cbb515d673037dca5fa680eedf86754a603` |
| 初始 x | `0.0` |
| 初始 y | `1.0` |
| 初始 yaw | `0.0` |
| 机器人出生点是否一致 | 是，Nav2 初始位姿与 Gazebo 出生点配置一致 |
| 每轮是否重启 Gazebo | 本次仅记录一轮，无法从 bag 独立验证 |

### 6.2 固定目标序列

四元数只需记录二维旋转使用的 `z` 和 `w`。

目标文件：`bags/slam_input_20260714_152204/waypoint2.yaml`。本次使用 `Start Waypoint Following`，每个点由独立的 `NavigateToPose` 任务执行。

| 序号 | 场景 | x | y | qz | qw |
| --- | --- | --- | --- | --- | --- |
| 1 | waypoint0 | 1.082658 | 1.089914 | 0.693815 | 0.720153 |
| 2 | waypoint1 | 0.855431 | 4.872472 | -0.999997 | 0.002501 |
| 3 | waypoint2 | -1.866043 | 4.661156 | -0.993002 | 0.118099 |
| 4 | waypoint3 | -4.150671 | 4.392479 | -0.717590 | 0.696466 |
| 5 | waypoint4 | -4.315859 | 0.217190 | -0.656362 | 0.754446 |
| 6 | waypoint5 | -4.228105 | -2.601277 | -0.999796 | 0.020209 |
| 7 | waypoint6 | -4.921895 | -1.307624 | 0.614371 | 0.789017 |
| 8 | waypoint7 | -3.235603 | 3.216552 | -0.012305 | 0.999924 |
| 9 | waypoint8 | -0.690890 | 2.877030 | -0.594496 | 0.804099 |
| 10 | waypoint9 | 0.004171 | 0.989207 | 0.013366 | 0.999911 |

## 7. NavFn + DWB 基线运行

### 7.1 运行元数据

| 项目 | 记录 |
| --- | --- |
| Profile | `navfn_dwb` |
| Run ID | `20260716-cartographer-waypoints` |
| 导航模式 | `Start Waypoint Following` |
| Bag 路径 | `bags/slam_input_20260714_152204/cartographer_nav_waypoints` |
| Bag 大小 | 32.2 MiB（MCAP 文件 33,798,063 bytes） |
| Bag 持续时间 | 173.336 s |
| Bag 消息数 | 223,625 |
| FollowWaypoints 任务时间 | 167.065 s |
| NavigateToPose 子任务 | 10 个，全部 `SUCCEEDED` |
| 参数文件 SHA-256 | `a659122a28615830360886dd20c419d8388cbcd3b97018020821878475370e6a` |
| Git commit | `eeaa8067cb43055d482e96088fd3ae21f35d35a6` |
| metadata.yaml SHA-256 | `d9341c0f37ca3c9e92b6f6b50a0a229b8a263b6fcdbcdbe0bdff35fa31a8d495` |
| MCAP SHA-256 | `464fe3c39795b5cfd82ad811cdd2cbe27fe828d90a55336eb11522bc08e60ce3` |
| 启动时间 | 仿真时间 19.689 s |
| 结束时间 | 仿真时间 193.025 s；文件写入完成于 2026-07-16 10:07:48 +08:00 |

### 7.2 每个目标结果

| 目标 | 结果 | 耗时/s | 恢复次数 | 碰撞 | 最终 x | 最终 y | 最终 yaw | 位置误差/m | 角度误差/rad | 备注 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| waypoint0 | `SUCCEEDED` | 9.432 | 0 | 未记录 | 1.059426 | 1.046965 | 1.329332 | 0.049 | 0.204 | 最后一条 feedback 略早于成功判定 |
| waypoint1 | `SUCCEEDED` | 21.188 | 0 | 未记录 | 0.860330 | 4.833052 | 2.938428 | 0.040 | 0.198 |  |
| waypoint2 | `SUCCEEDED` | 11.895 | 0 | 未记录 | -1.741475 | 4.734032 | -2.829831 | 0.144 | 0.075 | 位置误差接近 0.15 m 容差 |
| waypoint3 | `SUCCEEDED` | 16.428 | 0 | 未记录 | -4.109044 | 4.418814 | -1.790237 | 0.049 | 0.190 | 终点附近出现角速度反向调姿 |
| waypoint4 | `SUCCEEDED` | 19.242 | 0 | 未记录 | -4.313587 | 0.266290 | -1.629568 | 0.049 | 0.198 |  |
| waypoint5 | `SUCCEEDED` | 17.732 | 0 | 未记录 | -4.233786 | -2.584833 | -2.917157 | 0.017 | 0.184 |  |
| waypoint6 | `SUCCEEDED` | 11.037 | 0 | 未记录 | -4.892388 | -1.313499 | 1.506644 | 0.030 | 0.183 |  |
| waypoint7 | `SUCCEEDED` | 22.638 | 0 | 未记录 | -3.259842 | 3.182400 | 0.165869 | 0.042 | 0.190 | 窄通道路段未触发 recovery |
| waypoint8 | `SUCCEEDED` | 16.919 | 0 | 未记录 | -0.703229 | 2.843671 | -1.074974 | 0.036 | 0.198 | 终点附近出现角速度反向调姿 |
| waypoint9 | `SUCCEEDED` | 13.994 | 0 | 未记录 | 0.000715 | 1.025660 | -0.167933 | 0.037 | 0.195 | 正常返回起点附近 |

最终位姿和误差来自每个 `NavigateToPose` 目标最后一条 feedback，是成功判定前的近似值，不是 Gazebo ground truth。

### 7.3 运行分析

- 10 个目标全部成功，纯导航时间合计 160.505 s，按表中数值计算平均 16.051 s；包含 waypoint 任务切换和停留的完整任务时间为 167.065 s。
- 估算总行驶距离为 28.44 m。waypoint7 路段规划长度约 4.892 m、估算行驶距离约 4.935 m，没有恢复、停滞或持续振荡。
- 按“低速状态下角速度连续反向至少 3 次”统计到 2 次调姿，分别位于 waypoint3 和 waypoint8，发生时剩余距离约 0.082 m 和 0.075 m，均属于终点角度对准。
- AMCL 平均标准差约为 x=0.073 m、y=0.081 m、yaw=0.091 rad。`map -> odom` 最大单次位置修正为 0.121 m，较大的修正集中在 waypoint5 到 waypoint6。
- Bag 未记录 `/odom`、Gazebo ground truth、碰撞接触话题和 action result 服务事件，因此不能据此计算真实物理误差、确认碰撞次数或读取 `missed_waypoints` result。

## 8. Smac2D + MPPI 升级运行

### 8.1 运行元数据

| 项目 | 记录 |
| --- | --- |
| Profile | `smac_mppi` |
| Run ID |  |
| Bag 路径 |  |
| Bag 大小 |  |
| Bag 持续时间 |  |
| 参数文件 SHA-256 |  |
| Git commit |  |
| 启动时间 |  |
| 结束时间 |  |

### 8.2 每个目标结果

| 目标 | 结果 | 耗时/s | 恢复次数 | 碰撞 | 最终 x | 最终 y | 最终 yaw | 位置误差/m | 角度误差/rad | 备注 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 |  |  |  |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |  |  |  |  |
| 6 |  |  |  |  |  |  |  |  |  |  |
| 7 |  |  |  |  |  |  |  |  |  |  |
| 8 |  |  |  |  |  |  |  |  |  |  |
| 9 |  |  |  |  |  |  |  |  |  |  |
| 10 |  |  |  |  |  |  |  |  |  |  |

## 9. 导航结果汇总

| 指标 | NavFn + DWB | Smac2D + MPPI | 验收要求 |
| --- | --- | --- | --- |
| 成功数 / 10 | 10/10 |  | 至少 9/10，且不低于基线 |
| 碰撞次数 | 未记录，无法判定 |  | 0 |
| 恢复行为次数 | 0 |  | 记录并解释 |
| 振荡次数 | 2 次终点调姿；0 次持续振荡 |  | 无持续振荡 |
| 平均到达时间/s | 16.051（纯导航） |  | 不强制，作为对比 |
| 最大位置误差/m | 0.144（waypoint2） |  | 不超过 0.15 |
| 最大角度误差/rad | 约 0.204（最后一条 feedback） |  | 不超过 0.2 |

最终判断：

- [ ] Smac2D + MPPI 达到至少 9/10 的成功率。
- [ ] Smac2D + MPPI 没有发生碰撞。
- [ ] 所有成功目标满足位置和角度容差。
- [ ] 新方案没有增加持续振荡或规划死循环。
- [ ] 新方案总体表现不低于基线。

结论及证据：

```text
NavFn + DWB 基线在 cartographer_nav_waypoints 中完成 10/10 个目标，未触发恢复行为。
waypoint7 通过窄通道时没有复现 NavigateThroughPoses 下的原地 recovery。
最大位置误差为 waypoint2 的 0.144 m；最大记录角度误差来自成功判定前的最后一条 feedback，
不能作为最终 GoalChecker 采样值。碰撞次数因缺少接触或 ground truth 话题而无法判定。
```

## 10. 参数调整记录

每次只修改一组参数。未完成相同目标序列前，不进行下一次调整。

| 轮次 | Git commit | 参数文件 SHA-256 | 修改参数 | 修改前 | 修改后 | 原因 | 结果 | 保留/撤销 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 |  |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |  |

## 11. 异常与复现步骤

### 异常编号：EXP-001

| 项目 | 记录 |
| --- | --- |
| 现象 |  |
| 首次出现时间 |  |
| 对应 bag |  |
| Git commit |  |
| 参数文件 SHA-256 |  |
| 目标编号 |  |
| 日志摘要 |  |
| 根因 |  |
| 处理方式 |  |
| 修复后是否复现 |  |

最小复现步骤：

```text
1.
2.
3.
```

## 12. 实验完成检查

- [ ] 已记录 Git 分支、commit 和工作区状态。
- [ ] 已记录 ROS、Gazebo、机器人型号和 world。
- [x] 已记录地图和参数文件 SHA-256。
- [x] 已使用 `ros2 bag info` 检查全部 bag。
- [ ] 已记录原始 SLAM 输入 bag 的文件校验值。
- [ ] 两种 SLAM 使用同一个原始输入 bag。
- [ ] 两套 Nav2 使用相同地图、初始位姿和目标序列。
- [ ] 每套 Nav2 都在全新的 Gazebo 实例中独立运行。
- [x] 已填写每个目标结果和最终误差。
- [ ] 已记录异常、恢复行为和碰撞。
- [x] 结论能够追溯到具体 bag、commit 和参数校验值。
