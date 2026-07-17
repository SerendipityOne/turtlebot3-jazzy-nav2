# TurtleBot3 SLAM 与 Nav2 实验记录

本文用于记录 [Slam Toolbox 建图到 Nav2 优化全流程](./slam-toolbox-nav2-workflow.md) 和 [2D SLAM 与 Nav2 优化学习指南](./2d-slam-nav2-learning-guide.md) 中的实验。每轮实验开始前填写环境和输入信息，结束后填写结果与结论。

开始新一轮独立实验时，可以复制本文并使用日期命名：

```bash
cp docs/experiment-record.md docs/experiment-record-YYYYMMDD.md
```

不要把 `$HOME/bags/` 下的大型 bag 提交到 Git。仓库只保存 bag 名称、校验值、对应提交和实验结果。

## 1. 实验标识

| 项目 | 记录 |
| --- | --- |
| 实验名称 |  |
| 实验日期与时区 |  |
| 操作者 |  |
| 实验目的 |  |
| Git 分支 |  |
| Git commit |  |
| 工作区是否干净 |  |

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
| 操作系统 | Ubuntu 24.04 / 其他： |
| ROS 发行版 | Jazzy |
| RMW 实现 |  |
| Gazebo 版本 |  |
| `turtlebot3` 包版本 |  |
| `slam_toolbox` 包版本 |  |
| `navigation2` 包版本 |  |
| `TURTLEBOT3_MODEL` | burger |
| Gazebo world | `turtlebot3_house` |
| CPU |  |
| 内存 |  |

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
| `my_burger_dwb_navfn.yaml` |  |
| `my_burger.yaml` |  |
| `slam_toolbox.yaml` |  |
| `tb3_house.yaml` |  |
| `tb3_house.pgm` |  |

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
| Bag 名称 | `slam_input_YYYYMMDD_HHMMSS` |
| 完整路径 |  |
| 存储格式 | MCAP |
| 文件大小 |  |
| 开始时间 |  |
| 持续时间 |  |
| `/scan` 消息数 |  |
| `/odom` 消息数 |  |
| `/tf` 消息数 |  |
| `/tf_static` 消息数 |  |
| 录制时是否未启动 SLAM/Nav2 | 是 / 否 |
| 探索路线说明 |  |

采集命令：

```bash
export SLAM_INPUT_BAG="$HOME/bags/tb3_jazzy/slam_input_YYYYMMDD_HHMMSS"
du -sh "$SLAM_INPUT_BAG"
ros2 bag info --sort name "$SLAM_INPUT_BAG"
find "$SLAM_INPUT_BAG" -maxdepth 1 -type f -print0 \
  | sort -z \
  | xargs -0 sha256sum
```

Bag 文件校验值：

```text
在此粘贴 metadata.yaml 和 MCAP 文件的 SHA-256
```

### 4.2 输入完整性检查

- [ ] 包含 `/clock`。
- [ ] 包含 `/scan` 且消息数大于零。
- [ ] 包含 `/odom` 且消息数大于零。
- [ ] 包含 `/tf` 和 `/tf_static`。
- [ ] 不包含 `/map`。
- [ ] 录制时没有 Cartographer、Slam Toolbox、AMCL 或 Nav2 节点。
- [ ] 机器人完成预定路线并回到起点附近。

输入异常与处理：

```text
无 / 在此记录
```

## 5. SLAM A/B 结果

两种算法必须使用第 4 节中的同一个 bag，并使用相同的 `--clock 50 --rate 1.0 --delay 2` 播放参数。

### 5.1 Cartographer

| 项目 | 记录 |
| --- | --- |
| 输入 bag |  |
| 配置文件 | `turtlebot3_lds_2d.lua` |
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
| 地图路径 |  |
| 地图 SHA-256 |  |
| 初始 x |  |
| 初始 y |  |
| 初始 yaw |  |
| 机器人出生点是否一致 | 是 / 否 |
| 每轮是否重启 Gazebo | 是 / 否 |

### 6.2 固定目标序列

四元数只需记录二维旋转使用的 `z` 和 `w`。

| 序号 | 场景 | x | y | qz | qw |
| --- | --- | --- | --- | --- | --- |
| 1 | 长直道 |  |  |  |  |
| 2 | 90 度转弯 |  |  |  |  |
| 3 | 狭窄通道 |  |  |  |  |
| 4 | 靠近墙体 |  |  |  |  |
| 5 | 原地调整方向 |  |  |  |  |
| 6 |  |  |  |  |  |
| 7 |  |  |  |  |  |
| 8 |  |  |  |  |  |
| 9 |  |  |  |  |  |
| 10 |  |  |  |  |  |

## 7. NavFn + DWB 基线运行

### 7.1 运行元数据

| 项目 | 记录 |
| --- | --- |
| Profile | `navfn_dwb` |
| Run ID |  |
| Bag 路径 |  |
| Bag 大小 |  |
| Bag 持续时间 |  |
| 参数文件 SHA-256 |  |
| Git commit |  |
| 启动时间 |  |
| 结束时间 |  |

### 7.2 每个目标结果

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
| 成功数 / 10 |  |  | 至少 9/10，且不低于基线 |
| 碰撞次数 |  |  | 0 |
| 恢复行为次数 |  |  | 记录并解释 |
| 振荡次数 |  |  | 无持续振荡 |
| 平均到达时间/s |  |  | 不强制，作为对比 |
| 最大位置误差/m |  |  | 不超过 0.15 |
| 最大角度误差/rad |  |  | 不超过 0.2 |

最终判断：

- [ ] Smac2D + MPPI 达到至少 9/10 的成功率。
- [ ] Smac2D + MPPI 没有发生碰撞。
- [ ] 所有成功目标满足位置和角度容差。
- [ ] 新方案没有增加持续振荡或规划死循环。
- [ ] 新方案总体表现不低于基线。

结论及证据：

```text
在此引用具体目标编号、bag 路径、参数校验值和观察结果
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
- [ ] 已记录地图和参数文件 SHA-256。
- [ ] 已使用 `ros2 bag info` 检查全部 bag。
- [ ] 已记录原始 SLAM 输入 bag 的文件校验值。
- [ ] 两种 SLAM 使用同一个原始输入 bag。
- [ ] 两套 Nav2 使用相同地图、初始位姿和目标序列。
- [ ] 每套 Nav2 都在全新的 Gazebo 实例中独立运行。
- [ ] 已填写每个目标结果和最终误差。
- [ ] 已记录异常、恢复行为和碰撞。
- [ ] 结论能够追溯到具体 bag、commit 和参数校验值。
