# TurtleBot3 具身找物与 SmolVLA 实现流程

本文档对应 `feature/embodied-object-navigation` 分支，目标是在既有 Slam Toolbox + Nav2 系统上实现“去厨房找红色杯子”一类具身找物任务。ROS 节点使用 C++，ROS 2 launch 使用 Python；YOLOX 训练和 SmolVLA 单独使用 conda，不在 conda 中运行 ROS 2 Jazzy。

## 1. 当前实现边界

已实现：

- Burger 仿真模型的 640x480、15 Hz RGB-D 相机、点云和完整 TF。
- `FindObject` action 接口。
- OpenCV DNN + YOLOX ONNX 的 RGB-D 物体检测节点。
- 类别、颜色白名单和 OpenAI 兼容语言接口解析。
- 搜索视点排序、Nav2 `NavigateToPose`、0.8 m 接近位姿和到达后复核。
- 目标确认规则：最近 15 帧内至少匹配 7 帧。
- YOLOX 和 SmolVLA 两套独立 conda 环境。
- SmolVLA 五种高层技能的本地 HTTP 推理入口。

需要通过实验产生、不会提交到 Git：

- 有明确许可的杯子、瓶子、背包、球和箱子模型资产。
- 10,000 帧 COCO 格式训练集及其训练结果。
- YOLOX-Nano `.pth` 和 `.onnx` 权重。
- 500 回合 SmolVLA 数据、微调权重和真实成功率。

不应在没有真实实验数据时填写成功率、误检率或 VLA 对比结论。

## 2. 系统结构

```text
自然语言指令
  -> /find_object action
  -> OpenAI 兼容接口：类别/颜色/房间 JSON
  -> 语义搜索视点
  -> Nav2 NavigateToPose
  -> RGB + Depth + CameraInfo
  -> YOLOX-Nano + 深度中值 + TF(map)
  -> 最近 15 帧至少 7 帧确认
  -> 距目标 0.8 m 的接近位姿
  -> 到达后再次确认
  -> FindObject result

SmolVLA（实验支路）
  -> 仅选择高层技能
  -> go_to_viewpoint / rotate_scan / approach_target / report_not_found / stop
  -> 超时、低置信度或服务异常时回退确定性规则
```

Nav2 始终负责碰撞检查和底层运动控制。VLA 不直接发布 `/cmd_vel`。

## 3. 构建系统 ROS 包

在系统 shell 中执行，不激活 conda：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
source /opt/ros/jazzy/setup.bash
cd "$TB3_WS"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-up-to turtlebot3_embodied_navigation
source "$TB3_WS/install/setup.bash"
ros2 interface show turtlebot3_embodied_interfaces/action/FindObject
```

运行测试：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
source /opt/ros/jazzy/setup.bash
cd "$TB3_WS"
colcon test --packages-select turtlebot3_embodied_navigation
colcon test-result --verbose
```

## 4. 在线 SLAM 与 Nav2

### 4.1 启动 RGB-D Gazebo 模型

终端 1：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export TURTLEBOT3_MODEL=burger_cam
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

确认相机：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/camera_info --once
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

### 4.2 在线建图

按 [Slam Toolbox 建图到 Nav2 优化全流程](slam-toolbox-nav2-workflow.md) 完成在线建图、地图保存和验收。具身找物不使用建图 rosbag 代替在线探索。

### 4.3 启动 Nav2

停止 SLAM，但保持 Gazebo。终端 2：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export TURTLEBOT3_MODEL=burger_cam
export NAV_MAP="$TB3_WS/maps/slam_toolbox/house.yaml"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_navigation2 my_nav2.launch.py \
  use_sim_time:=true \
  map:="$NAV_MAP" \
  params_file:="$TB3_WS/src/turtlebot3/turtlebot3_navigation2/param/my_burger_smac2d_mppi.yaml"
```

先在 RViz 校验 AMCL 初始位姿、全局路径和窄通道通行，再启动找物任务。

## 5. YOLOX-Nano 训练与导出

### 5.1 创建隔离环境

该环境仅用于训练和导出，不能在其中 source ROS 2：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
conda env create -f "$TB3_WS/ml/yolox/environment.yml"
conda activate tb3-yolox-train
python --version
```

环境固定使用 YOLOX 0.3.0。官方项目采用 Apache-2.0，训练前仍需逐项记录仿真资产的来源、版本和许可证。

### 5.2 数据契约

数据集采用 COCO 目录结构：

```text
datasets/tb3_objects/
├── train2017/
├── val2017/
└── annotations/
    ├── instances_train.json
    └── instances_val.json
```

类别顺序必须与运行节点一致：

```text
0 cup
1 bottle
2 backpack
3 ball
4 box
```

数据生成要求：

- 目标为 10,000 帧，按场景随机种子划分训练/验证/测试，禁止将同一场景连续帧拆到不同集合。
- 颜色覆盖红、绿、蓝、黄，类别和颜色组合应近似均衡。
- 随机化目标位置、朝向、尺度、光照和相机距离，并保留遮挡与负样本。
- 每次生成保存场景 seed、资产清单、Git commit、Gazebo 版本和标注统计。
- 自动标注必须从 Gazebo segmentation/bounding-box ground truth 转换；禁止用待训练模型生成自己的真值。

当前仓库定义了训练和运行接口，但没有内置许可不明的第三方资产，也没有伪造 10,000 帧数据。完成资产清单和 Gazebo ground-truth 采集器后，才进入训练阶段。

### 5.3 训练

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export TB3_COCO_DATASET="$TB3_WS/datasets/tb3_objects"
export YOLOX_OUTPUT="$TB3_WS/outputs/yolox_nano_tb3"
conda activate tb3-yolox-train
cd "$TB3_WS"
python -m yolox.tools.train \
  -f "$TB3_WS/ml/yolox/exps/yolox_nano_tb3.py" \
  -d 1 -b 16 --fp16 -o \
  --experiment-name yolox_nano_tb3
```

训练后先对未见 seed 的测试集计算 COCO mAP、逐类召回率和颜色混淆，再导出 ONNX。最终阈值不能只按训练集调整。

### 5.4 导出 ONNX

YOLOX 0.3.0 的官方导出脚本位于其源码仓库。使用固定 tag 获取导出工具，不修改本项目 Git 历史：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export YOLOX_HOME="$TB3_WS/third_party/YOLOX"
export YOLOX_CHECKPOINT="$TB3_WS/outputs/yolox_nano_tb3/best_ckpt.pth"
export YOLOX_ONNX="$TB3_WS/artifacts/yolox_nano_tb3.onnx"
conda activate tb3-yolox-train
git clone --branch 0.3.0 --depth 1 \
  https://github.com/Megvii-BaseDetection/YOLOX.git "$YOLOX_HOME"
mkdir -p "$TB3_WS/artifacts"
cd "$YOLOX_HOME"
python tools/export_onnx.py \
  --output-name "$YOLOX_ONNX" \
  -f "$TB3_WS/ml/yolox/exps/yolox_nano_tb3.py" \
  -c "$YOLOX_CHECKPOINT" \
  --no-onnxsim
```

## 6. 启动感知与找物任务

语言接口采用 OpenAI 兼容的 `/chat/completions` 路径。密钥只放在当前 shell 环境中，不写 YAML、不写 rosbag、不提交 Git。

终端 3：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export YOLOX_ONNX="$TB3_WS/artifacts/yolox_nano_tb3.onnx"
export VLM_API_BASE="https://your-openai-compatible-host/v1"
export VLM_MODEL="your-model-name"
export VLM_API_KEY="replace-in-current-shell"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_embodied_navigation embodied_navigation.launch.py \
  model_path:="$YOLOX_ONNX" \
  use_sim_time:=true
```

检查节点：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 topic echo /embodied/detections
ros2 action info /find_object
```

发送任务：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 action send_goal \
  /find_object \
  turtlebot3_embodied_interfaces/action/FindObject \
  "{instruction: '去厨房找红色杯子'}" \
  --feedback
```

`find_object.yaml` 中的搜索视点来自当前 house 地图。换地图后必须重新采集 `x, y, yaw`，并同步修改 `viewpoint_rooms`。

## 7. 记录可复现实验

记录内容包含输入、感知输出、任务 action、Nav2 action、定位和运动控制。终端 4：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export RUN_ID="$(date +%Y%m%d_%H%M%S)"
export EMBODIED_BAG="$TB3_WS/bags/embodied_find_object_$RUN_ID"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 bag record \
  --use-sim-time \
  --storage mcap \
  --include-hidden-topics \
  --output "$EMBODIED_BAG" \
  --topics \
  /clock /tf /tf_static /map /amcl_pose /scan /odom /cmd_vel \
  /camera/color/image_raw /camera/depth/image_raw /camera/camera_info \
  /embodied/detections /embodied/debug_image \
  /find_object/_action/feedback /find_object/_action/status \
  /navigate_to_pose/_action/feedback /navigate_to_pose/_action/status
```

停止后立即检查：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export EMBODIED_BAG="$TB3_WS/bags/embodied_find_object_YYYYMMDD_HHMMSS"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 bag info --sort name "$EMBODIED_BAG"
```

每次实验至少记录：Git commit、地图 SHA-256、参数 SHA-256、场景 seed、目标真值、解析结果、搜索视点顺序、最终 action 状态、耗时、碰撞和失败原因。

## 8. SmolVLA 实验支路

### 8.1 创建独立环境

SmolVLA 不安装进 YOLOX 环境，也不在该环境中启动 ROS：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
conda env create -f "$TB3_WS/ml/smolvla/environment.yml"
conda activate tb3-smolvla
python -c "import lerobot; print(lerobot.__version__)"
```

### 8.2 数据与切分

动作向量是五维 one-hot，契约见 `ml/smolvla/README.md`。计划记录 500 回合，并按场景 seed 固定切分：350 训练、75 验证、75 测试。每回合至少包含前置 RGB 图像、机器人状态、自然语言任务、规则控制器选择的技能、执行结果和时间戳。

先用规则控制器生成演示数据；不得把测试 seed 放进训练集。模型需要与数据集完全一致的 `observation.images.front`、`observation.state` 和五维 `action` feature 定义。

### 8.3 微调

官方建议先从约 50 回合起步；本项目为多场景高层技能选择规划 500 回合。远程 GPU 命令：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export HF_USER="your-huggingface-user"
export LEROBOT_DATASET="$HF_USER/tb3_high_level_skills"
export SMOLVLA_OUTPUT="$TB3_WS/outputs/smolvla_tb3"
conda activate tb3-smolvla
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id="$LEROBOT_DATASET" \
  --batch_size=64 \
  --steps=20000 \
  --output_dir="$SMOLVLA_OUTPUT" \
  --job_name=smolvla_tb3 \
  --policy.device=cuda \
  --wandb.enable=true
```

### 8.4 启动本地策略服务

服务只绑定回环地址，ROS 侧以后通过 HTTP 使用，不在 conda 环境中导入 `rclpy`：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export SMOLVLA_POLICY_PATH="$TB3_WS/outputs/smolvla_tb3/checkpoints/last/pretrained_model"
export SMOLVLA_DEVICE="cuda"
conda activate tb3-smolvla
cd "$TB3_WS"
uvicorn ml.smolvla.policy_server:app \
  --host 127.0.0.1 \
  --port 8088
```

健康检查：

```bash
export SMOLVLA_POLICY_URL="http://127.0.0.1:8088"
curl --fail "$SMOLVLA_POLICY_URL/health"
```

当前 ROS 主线默认使用确定性搜索规则。只有在 50 个成对未见 seed 上完成 A/B 验证，并且 VLA 成功率不低于规则基线、碰撞为 0 后，才允许将 VLA 设为默认；服务超时、HTTP 错误、非法技能或低置信度必须回退规则。

## 9. 验收门槛

主线在至少 30 个未见场景 seed 上验收：

| 指标 | 门槛 |
| --- | --- |
| 找物任务成功率 | >= 80% |
| 碰撞 | 0 |
| 假阳性率 | <= 5% |
| 单任务总时长 | <= 300 s |
| 多帧确认 | 最近 15 帧至少 7 帧 |
| 接近距离 | 0.8 m，受 Nav2 可达性约束 |

失败必须归类为语言解析、目标未找到、导航失败、感知超时、总超时或取消，不能只记录“失败”。

## 10. 常见故障

- `ros2 run` 找不到节点：重新构建并 source 工作区，确认 `ros2 pkg executables turtlebot3_embodied_navigation`。
- 节点报告 `model_path` 无效：ONNX 权重不存在或不是普通文件；节点会 fail-fast。
- 一直没有 detection：检查 RGB、Depth、CameraInfo 的频率和 optical frame TF，再检查 ONNX 输出是否为 YOLOX 解码后的 `N x (5+C)`。
- 已检测但不能接近：检查目标点从 `camera_depth_optical_frame` 到 `map` 的 TF，以及接近位姿是否落在 inflation layer 内。
- 语言 action 立即失败：检查三个 `VLM_*` 环境变量；程序不会打印 API key。
- SmolVLA 服务启动失败：检查权重的 feature schema 是否与 `observation.images.front`、状态维度和五维 action 一致。
