# 分层 SmolVLA 找球与导航流程

本文档只描述当前可运行的 `continuous_local_goal` 流程。旧的 `discrete_skill` 数据、checkpoint 和训练流程已清理，不能再使用。

系统只在 `turtlebot3_house` Gazebo 仿真中验证，目标是红、绿、蓝、黄球。当前已训练模型为 `tb3-ball-continuous-v2`。

## 1. 系统边界

执行链路为：自然语言指令 → LLM 解析目标并在阶段结束时选择 `search`、`approach`、`verify`、`done` 或 `fail` → SmolVLA 根据 RGB、阶段指令和机器人状态生成局部目标 `[delta_x_m, delta_y_m, delta_yaw_rad]` → Nav2 执行导航。

- LLM 不输出坐标、路径或底盘速度。
- SmolVLA 局部目标限制为平移不超过 `0.75 m`、转角不超过 `π/4`；无有效进展的输出会被拒绝。
- 节点只把局部目标转换为 `map` 中的 `NavigateToPose`，绝不直接发布 `/cmd_vel`。
- 接近前必须满足语言颜色契约、YOLO 最近 15 帧至少命中 7 帧、检测不超过 1 秒且 Nav2 可达；任一条件不满足即安全停止。

`shadow` 模式仍按固定视点搜索，仅记录 VLA 局部目标；`active` 模式才允许连续 VLA 局部目标驱动 Nav2。

## 2. 构建与运行资产

先构建 ROS 节点。删除 `build/` 与 `install/` 后必须重新执行此步骤：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
cd "$TB3_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select turtlebot3_embodied_navigation turtlebot3_gazebo
source "$TB3_WS/install/setup.bash"
```

运行期需要以下本地、被 Git 忽略的资产：

| 资产 | 作用 |
| --- | --- |
| `artifacts/smolvlm2-500m-video-instruct/` | SmolVLM 基座；策略服务离线加载，不在运行期访问 Hugging Face。 |
| `artifacts/yolo11n_ball.onnx` | ball-only YOLO 安全确认模型。 |
| `outputs/smolvla_continuous_formal_20260730/checkpoints/010000/pretrained_model/` | 当前连续 VLA checkpoint。 |
| `datasets/gso_assets/models/` | 可选 Gazebo GSO 场景资产；仅在需要插入本地模型时使用。 |

本机 Python 环境为 `lerobot`。新终端需要先显式激活，不执行 `conda init`：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
cbase
conda activate lerobot
cd "$TB3_WS"
python --version
```

## 3. 连续 VLA 数据采集与门禁

连续 Oracle 保存有界局部目标、阶段指令、目标可见性和位姿审计字段。目标不可见时监督搜索最近未访问区域；目标可见时监督停在距球 `0.8 m` 外的接近目标。Oracle 真值不进入模型输入。

先启动 Gazebo，再运行 10 Episode Diagnostic；Diagnostic 只能排障，不能训练：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export TURTLEBOT3_MODEL=waffle_pi_cam
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_diagnostic_20260730"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_gazebo vla_oracle_dataset.launch.py \
  output_dir:="$VLA_RAW_DIR" \
  policy_interface:=continuous_local_goal \
  episode_count:=10 \
  target_absent_ratio:=0.30 \
  seed:=20260730 \
  preview:=false
```

立即运行数据门禁；非零退出时不得进入 Pilot：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_diagnostic_20260730"
export VLA_REPORT_DIR="$TB3_WS/datasets/vla_reports/continuous_diagnostic_20260730"
cd "$TB3_WS"
python3 tools/vla/validate_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" \
  --report-dir "$VLA_REPORT_DIR"
```

Pilot 使用同一命令，将目录改为 `continuous_pilot_20260730`、`episode_count` 改为 `200`。人工审阅 `report.md` 后，必须确认：目标存在 Episode 的最后一帧进入 `approach`；目标不存在 Episode 完成覆盖而不接近；所有局部目标都在边界内。

正式采集只能使用批准文件、固定 seed 和固定目录；存在同名目录时不要覆盖：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_formal_20260730"
export VLA_APPROVAL_FILE="$TB3_WS/datasets/vla_raw/continuous_formal_approval_20260730.txt"
mkdir -p "$TB3_WS/datasets/vla_raw"
printf 'APPROVED\n' > "$VLA_APPROVAL_FILE"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_gazebo vla_oracle_dataset.launch.py \
  output_dir:="$VLA_RAW_DIR" \
  approval_file:="$VLA_APPROVAL_FILE" \
  policy_interface:=continuous_local_goal \
  episode_count:=2000 \
  target_absent_ratio:=0.30 \
  seed:=20260730 \
  preview:=false
```

## 4. 验证、转换与训练

当前正式数据已经完成。重新转换或重训前，先重新验证，并只将 `train/` 用于训练：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_formal_20260730"
export VLA_REPORT_DIR="$TB3_WS/datasets/vla_reports/continuous_formal_20260730"
export VLA_LE_ROBOT_ROOT="$TB3_WS/datasets/vla_lerobot/continuous_formal_20260730"
cbase
conda activate lerobot
cd "$TB3_WS"
python3 tools/vla/validate_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" \
  --report-dir "$VLA_REPORT_DIR"
python tools/vla/convert_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" --validation-report "$VLA_REPORT_DIR/report.json" \
  --split train --output-root "$VLA_LE_ROBOT_ROOT/train" \
  --repo-id local/tb3_ball_continuous_vla_train
python tools/vla/convert_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" --validation-report "$VLA_REPORT_DIR/report.json" \
  --split validation --output-root "$VLA_LE_ROBOT_ROOT/validation" \
  --repo-id local/tb3_ball_continuous_vla_validation
python tools/vla/convert_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" --validation-report "$VLA_REPORT_DIR/report.json" \
  --split test --output-root "$VLA_LE_ROBOT_ROOT/test" \
  --repo-id local/tb3_ball_continuous_vla_test
```

训练使用连续回归，不使用离散动作均衡器。若保留现有 10k step checkpoint，不要对同一输出目录再次启动训练：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export VLA_CONTINUOUS_TRAIN_ROOT="$TB3_WS/datasets/vla_lerobot/continuous_formal_20260730/train"
export VLA_CONTINUOUS_OUTPUT_DIR="$TB3_WS/outputs/smolvla_continuous_formal_20260730"
export VLA_CONTINUOUS_TRAIN_REPO_ID=local/tb3_ball_continuous_vla_train
cbase
conda activate lerobot
cd "$TB3_WS"
bash tools/vla/train_continuous_smolvla.sh
```

## 5. Shadow 与 Active 验收

先在 Gazebo 放置红球。可用 GUI 的 Resource Spawner 插入 `datasets/gso_assets/models/` 中的本地资源作为干扰物；ball-only 检测器只把球当目标。GUI 插入不写回 world 文件，需记录模型名与位姿。

终端一启动 Gazebo：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export TURTLEBOT3_MODEL=waffle_pi_cam
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

终端二启动 Nav2：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export TURTLEBOT3_MODEL=waffle_pi_cam
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_navigation2 my_nav2.launch.py use_sim_time:=true
```

终端三启动本地策略服务。`HF_HUB_OFFLINE=1` 确保部署期不会联网：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export VLA_CHECKPOINT="$TB3_WS/outputs/smolvla_continuous_formal_20260730/checkpoints/010000/pretrained_model"
export VLA_BASE_MODEL_DIR="$TB3_WS/artifacts/smolvlm2-500m-video-instruct"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
cbase
conda activate lerobot
cd "$TB3_WS"
python tools/vla/vla_policy_server.py \
  --checkpoint "$VLA_CHECKPOINT" \
  --vlm-model-dir "$VLA_BASE_MODEL_DIR" \
  --interface continuous_local_goal \
  --device cuda --seed 20260730 --model-version tb3-ball-continuous-v2
```

终端四启动检测与找物服务。`.env` 中的 VLM 凭据只在本机加载，不能提交：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
export YOLO11_ONNX="$TB3_WS/artifacts/yolo11n_ball.onnx"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
source "$TB3_WS/.env"
ros2 launch turtlebot3_embodied_navigation embodied_navigation.launch.py \
  model_path:="$YOLO11_ONNX" use_sim_time:=true policy_mode:=shadow \
  policy_interface:=continuous_local_goal \
  continuous_policy_endpoint:=http://127.0.0.1:8089/predict_local_goal
```

终端五发送任务：

```bash
export TB3_WS="$HOME/GitHub/tb3_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 action send_goal /find_object \
  turtlebot3_embodied_interfaces/action/FindObject \
  "{instruction: '找红球'}" --feedback
```

Shadow 日志必须显示有界的 `VLA local goal dx/dy/yaw`，且 YOLO 7/15 确认、Nav2 与定位正常。满足后才将第四个终端的 `policy_mode:=shadow` 改为 `policy_mode:=active`。

验收标准：在 `turtlebot3_house` 的 10 次任务中至少 8 次成功；每次停在距球 `0.8 m` 范围内；零碰撞；发生检测、规划或策略错误时安全停止。

## 6. 故障定位顺序

1. 策略服务返回 4xx/5xx：检查 checkpoint、Conda 环境、本地 SmolVLM 路径、状态契约和 JPEG 输入。
2. 接近被拒：检查语言颜色、YOLO 最近 15 帧中的 7 帧确认与检测时效；这属于预期安全拦截。
3. Nav2 失败：检查 `map -> odom -> base_link`、costmap 和目标可达性；不要让 VLA 绕过 Nav2。
4. VLM 解析失败：检查 `.env` 的 `VLM_API_BASE`、`VLM_MODEL`、`VLM_API_KEY`，不要在终端或日志中打印密钥。
