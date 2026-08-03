# 分层 SmolVLA 找球与导航流程

本文档保留已验证的离散技能基线，并增加连续局部目标 v2。两者只在 `turtlebot3_house` 的 Gazebo 仿真中验证，目标类别为红、绿、蓝、黄球。

第 1～8 节描述 `discrete_skill` 基线；第 9 节是新的 `continuous_local_goal` 通道，两者的数据、checkpoint 与运行端点不能混用。

## 1. 系统边界

SmolVLA 不输出 `/cmd_vel`，只从 10 个高层动作中选一个：三个相对候选视点、左右扫描、四种颜色的接近、报告未找到。Nav2 仍负责路径规划、避障、恢复与底盘控制。

每次决策输入为原始中文指令、前置 RGB 图像和 23 维状态：

- 三个最近未访问视点的 `[距离, sin(相对方位), cos(相对方位), 有效标记]`；
- 上一次动作的 10 维 one-hot；
- 当前视点扫描进度 `0 / 0.5 / 1`。

动作向量仍是 10 维 one-hot，但训练保持 SmolVLA 原生的连续动作流匹配；运行时按与 10 个 one-hot 原型的最近距离解码。因此 `prototype_distance` 与 `prototype_gap` 是连续输出的安全门限，不是分类概率。

语言模型仅独立解析“目标类别/颜色”安全契约。它不把解析结果、Gazebo 真实标签或目标颜色注入 VLA 输入。VLA 的 `approach_<color>` 只有在颜色等于语言契约、YOLO 在最近 15 帧至少命中 7 帧、最新检测不超过 1 秒且 Nav2 可执行时才会放行。

`active` 模式不会退回固定搜索：策略服务、图像、状态、模型版本或门限任一异常都会失败关闭。`shadow` 模式只记录决策，不改变现有固定搜索的动作。

## 2. 构建与 Conda 环境

先构建新增的 ROS 节点和启动文件：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
cd "$TB3_WS"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select turtlebot3_embodied_navigation turtlebot3_gazebo
source install/setup.bash
```

创建隔离的训练/推理环境；不要执行 `conda init` 或让 base 自动激活：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export CONDA_ROOT=/path/to/miniconda3
cd "$TB3_WS"
source "$CONDA_ROOT/etc/profile.d/conda.sh"
conda env create -f tools/vla/environment.yml
conda activate tb3-vla
```

## 3. 诊断与 Pilot Oracle 数据

生成器在每个 Episode 中重置 Waffle Pi Cam 位姿、放置彩球并读取 Gazebo instance-segmentation 标签。输出的是未转换原始数据：JPEG、状态、动作、指令和 Oracle 检查字段；`datasets/` 已被 Git 忽略。`target_label_pixels` 是目标语义标签的像素数；它必须与 `target_visible` 一致。只要目标语义可见，即使目标较远且像素面积较小，也应输出对应颜色的 `approach_*`。

先运行 10 Episode 诊断。目标存在时，生成器必须在完整覆盖前输出对应颜色且语义可见的 `approach_*`；若覆盖完成仍未出现可见目标，生成器会失败关闭。诊断数据只用于排障，不能转换或训练：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/diagnostic_20260729"
cd "$TB3_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot3_gazebo vla_oracle_dataset.launch.py \
  output_dir:="$VLA_RAW_DIR" \
  episode_count:=10 \
  target_absent_ratio:=0.30 \
  seed:=20260729 \
  preview:=false
```

诊断完成后立即运行同一套门禁；任一目标存在 Episode 未以可见且颜色匹配的 `approach_*` 终止时，命令会返回非零，必须先修复生成器：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/diagnostic_20260729"
export VLA_REPORT_DIR="$TB3_WS/datasets/vla_reports/diagnostic_20260729"
cd "$TB3_WS"
python3 tools/vla/validate_raw_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" \
  --report-dir "$VLA_REPORT_DIR"
```

诊断无 Oracle 失败后，再生成 200 个 Pilot Episode。Pilot 仅用于数据门禁，目标 70%、未找到目标 30%，并按 Episode 固定切分为 70% / 15% / 15%。输出目录必须不存在，生成器会拒绝覆盖：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/pilot_20260729"
cd "$TB3_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot3_gazebo vla_oracle_dataset.launch.py \
  output_dir:="$VLA_RAW_DIR" \
  episode_count:=200 \
  target_absent_ratio:=0.30 \
  seed:=20260729 \
  preview:=false
```

该命令由你在终端运行；完成后启动文件会随生成器退出而关闭 Gazebo。要观察单 Episode 布局可改为 `preview:=true episode_count:=1`，但不要把预览过程的数据当正式数据集。`pilot_20260727` 是 Oracle 故障样本；`pilot_20260728` 与 `formal_20260728` 使用当前“语义可见即可接近”的规则，能够作为本轮数据门禁与正式候选数据。`pilot_20260729` 保留作较严格近距离样本的诊断对照。

## 4. Pilot 数据门禁

校验器会单独创建 `report.md` 和 `report.json`，校验 23 维状态、候选槽位、左右扫描顺序、终止动作、图像存在性、语义可见性与 70/15/15 划分。`approach_*` 只要求目标语义可见；实际运行仍由 YOLO 最近 15 帧至少命中 7 帧、最新检测时效和 Nav2 可达性共同决定是否放行：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/pilot_20260728"
export VLA_REPORT_DIR="$TB3_WS/datasets/vla_reports/pilot_20260728"
cd "$TB3_WS"
python3 tools/vla/validate_raw_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" \
  --report-dir "$VLA_REPORT_DIR"
```

Pilot 仅在命令返回 `0` 且人工审阅 `report.md` 后进入人工门禁。报告必须显示：每个目标存在 Episode 都以颜色匹配的可见 `approach_*` 终止；每个目标不存在 Episode 都以完整覆盖后的 `report_not_found` 终止。Pilot 不转换、不训练。本轮 `pilot_20260728` 已通过自动门禁，仍须完成报告与样本图像人工审阅。

## 5. 正式 2000 Episode 采集

只有新的 Pilot 报告通过、人工审阅完成后，才创建批准文件并生成新的正式数据。首次运行时正式目录必须是新的空目录；若中断，可使用 `resume:=true` 只跳过连续且完整的 Episode。恢复不会覆盖或删除不完整 Episode，必须先人工确认其处理方式。本轮 `formal_20260728` 已完成采集，无需因远距离可见目标而重采。

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/formal_20260729"
export VLA_APPROVAL_FILE="$TB3_WS/datasets/vla_raw/formal_approval_20260729.txt"
cd "$TB3_WS"
mkdir -p "$TB3_WS/datasets/vla_raw"
printf 'APPROVED\n' > "$VLA_APPROVAL_FILE"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot3_gazebo vla_oracle_dataset.launch.py \
  output_dir:="$VLA_RAW_DIR" \
  approval_file:="$VLA_APPROVAL_FILE" \
  episode_count:=2000 \
  target_absent_ratio:=0.30 \
  seed:=20260730 \
  preview:=false
```

恢复示例：必须使用与首次完全相同的输出目录、批准文件、Episode 总数和 seed。若任何已有 Episode 缺少元数据、步骤或引用图像，生成器会拒绝启动而不是覆盖数据。

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/formal_20260729"
export VLA_APPROVAL_FILE="$TB3_WS/datasets/vla_raw/formal_approval_20260729.txt"
cd "$TB3_WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch turtlebot3_gazebo vla_oracle_dataset.launch.py \
  output_dir:="$VLA_RAW_DIR" \
  approval_file:="$VLA_APPROVAL_FILE" \
  episode_count:=2000 \
  target_absent_ratio:=0.30 \
  seed:=20260730 \
  resume:=true \
  preview:=false
```

## 6. 正式数据门禁与转换

正式集也必须重新校验。转换器会核对报告版本、原始目录和清单摘要；任何原始标签在校验后被修改、报告不通过或报告指向其他目录，都会在创建 LeRobot 输出前失败。`formal_20260728` 已按“语义可见即可接近”的规则通过自动门禁；完成本节报告与样本图像人工审阅后，可转换和训练，无需重新完成 Diagnostic、Pilot 与 Formal 流程。

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/formal_20260728"
export VLA_REPORT_DIR="$TB3_WS/datasets/vla_reports/formal_20260728"
cd "$TB3_WS"
python3 tools/vla/validate_raw_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" \
  --report-dir "$VLA_REPORT_DIR"
```

仅当命令返回 `0` 且人工审阅 `report.md` 后，才转换正式数据。转换器拒绝覆盖既有输出；`--overwrite` 会递归删除输出目录，因此不要把它放进默认流程。

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export CONDA_ROOT=/path/to/miniconda3
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/formal_20260728"
export VLA_REPORT_DIR="$TB3_WS/datasets/vla_reports/formal_20260728"
export VLA_LE_ROBOT_ROOT="$TB3_WS/datasets/vla_lerobot/formal_20260728"
cd "$TB3_WS"
"$CONDA_ROOT/bin/conda" run -n tb3-vla python tools/vla/convert_raw_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" --validation-report "$VLA_REPORT_DIR/report.json" --split train \
  --output-root "$VLA_LE_ROBOT_ROOT/train" \
  --repo-id local/tb3_ball_vla_formal_train
"$CONDA_ROOT/bin/conda" run -n tb3-vla python tools/vla/convert_raw_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" --validation-report "$VLA_REPORT_DIR/report.json" --split validation \
  --output-root "$VLA_LE_ROBOT_ROOT/validation" \
  --repo-id local/tb3_ball_vla_formal_validation
"$CONDA_ROOT/bin/conda" run -n tb3-vla python tools/vla/convert_raw_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" --validation-report "$VLA_REPORT_DIR/report.json" --split test \
  --output-root "$VLA_LE_ROBOT_ROOT/test" \
  --repo-id local/tb3_ball_vla_formal_test
```

## 7. 云端训练与门限校准

把已通过门禁的 `train/` 数据和同版本代码复制到云端 GPU。云端使用锁定的 `lerobot[smolvla]==0.6.1` 环境，默认只训练 expert、冻结视觉编码器、`chunk_size=1` 和 `n_action_steps=1`。项目训练入口把 `policy.input_features` 设为 `null`，由 LeRobot 从本项目单路 `observation.images.front` 数据集元数据推导输入特征，避免沿用基座的三相机特征名。它使用 LeRobot 的逐样本 loss 加权接口：从训练集 `conversion_report.json` 读取十个动作计数，采用上限为 4 倍的逆平方根权重并归一化为原始数据均值 1。它只允许 `train/`，验证和测试集保持原始分布。训练显式关闭模型和检查点的 Hugging Face 上传，全部产物只写入 `VLA_OUTPUT_DIR`：

```bash
export VLA_TRAIN_ROOT=/workspace/tb3/datasets/vla_lerobot/formal_20260728/train
export VLA_OUTPUT_DIR=/workspace/tb3/outputs/smolvla_formal_20260728
export VLA_TRAIN_REPO_ID=local/tb3_ball_vla_formal_train
export CONDA_ROOT=/path/to/miniconda3
cd /workspace/tb3
source "$CONDA_ROOT/etc/profile.d/conda.sh"
conda activate tb3-vla
bash tools/vla/train_smolvla.sh
```

`tools/vla/train_smolvla_balanced.py` 会在启动时检查 LeRobot 版本、`chunk_size=1` 的动作形状和训练集转换报告；任一条件不满足即失败，不会退回无权重训练。

训练后在 validation/test 上记录每个动作的最近原型距离与第一、第二近原型的差值。只有在这些数据上确定了 `policy_max_prototype_distance` 和 `policy_min_prototype_gap`，并确认没有错误接近动作，才可进入 shadow。v1 不假设任意固定阈值能跨模型复用。

## 8. 仿真端到端验证：放置模型、Shadow 与 Active

这一节验证真实运行效果，而非只看离线动作一致率。SmolVLA 只选择高层技能，不能直接发布 `/cmd_vel`；YOLO 的最近 15 帧至少 7 帧确认、语言颜色契约和 Nav2 可达性仍是接近动作的强制门禁。当前训练产物使用单路 `observation.images.front`、23 维状态和 10 维动作，运行时必须保持相同契约。

### 8.1 运行前门禁

当前本机训练完成的模型在 `010000` checkpoint。`yolo11n_ball.onnx` 是 ball-only 运行时安全确认模型；缺少它时不得启动端到端找球，也不要通过放宽阈值绕过该门禁。

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export CONDA_ROOT=$HOME/miniconda3
export VLA_CONDA_ENV=lerobot
export VLA_CHECKPOINT="$TB3_WS/outputs/smolvla_formal_20260728/checkpoints/010000/pretrained_model"
export YOLO11_ONNX="$TB3_WS/models/yolo11n_ball.onnx"
test -s "$VLA_CHECKPOINT/model.safetensors"
test -s "$YOLO11_ONNX"
```

`VLA_CONDA_ENV=lerobot` 是本机完成本轮训练的环境名。若在另一台机器复现，环境名可以不同，但必须安装锁定的 LeRobot/SmolVLA 依赖并能加载该 checkpoint。

### 8.1 基座模型一次性下载

训练 checkpoint 不包含完整的 SmolVLM 基座。首次部署时，把固定版本的基座下载到项目的 Git 忽略目录；运行期只接受这个本地目录，绝不访问 Hugging Face。下面命令只需执行一次。若该机器需要代理访问 Hugging Face，只在此下载终端临时设置代理即可，后续运行策略服务不需要代理。

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export CONDA_ROOT=$HOME/miniconda3
export VLA_CONDA_ENV=lerobot
export VLA_BASE_MODEL_ID=HuggingFaceTB/SmolVLM2-500M-Video-Instruct
export VLA_BASE_MODEL_REVISION=7b375e1b73b11138ff12fe22c8f2822d8fe03467
export VLA_BASE_MODEL_DIR="$TB3_WS/artifacts/smolvlm2-500m-video-instruct"
mkdir -p "$VLA_BASE_MODEL_DIR"
"$CONDA_ROOT/bin/conda" run -n "$VLA_CONDA_ENV" hf download \
  "$VLA_BASE_MODEL_ID" \
  --revision "$VLA_BASE_MODEL_REVISION" \
  --local-dir "$VLA_BASE_MODEL_DIR"
test -s "$VLA_BASE_MODEL_DIR/config.json"
test -s "$VLA_BASE_MODEL_DIR/model.safetensors"
```

### 8.2 在 Gazebo 世界中放置目标与干扰物

`turtlebot3_house.launch.py` 会把项目内的 `turtlebot3_gazebo/models` 加入 `GZ_SIM_RESOURCE_PATH`。项目下载的 GSO 资产位于被 Git 忽略的 `datasets/gso_assets/models`，每个可插入模型目录必须包含 `model.config` 和 `model.sdf`。把该目录在启动 Gazebo 前也加入资源路径：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export GZ_ASSET_ROOT="$TB3_WS/datasets/gso_assets/models"
export GZ_SIM_RESOURCE_PATH="$GZ_ASSET_ROOT${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi_cam
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

临时手动放置已有本地模型时，在 Gazebo GUI 右上角的 **Plugins** 菜单打开 **Resource Spawner**。在 **Local resources** 中选择模型，单击后在地面上再次单击完成插入；使用 **Transform Control** 或 Component Inspector 调整位置和朝向。GSO 的 cup、box、backpack 等仅可作为干扰物，不能充当 ball-only 检测器的目标。GUI 插入不会写回 world 文件，因此每次实验都要记录模型名与 `x/y/z/yaw`。

需要可复现地放置球时，不依赖 GUI，另开终端向当前 `default` world 的创建服务发送与 Oracle 数据同尺寸的静态红球。先确认 `/world/default/create` 存在；球的半径为 `0.10 m`，因此 `z=0.10` 使其落在地面上。将 `x/y` 改为可达且不在初始相机视野内的位置：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export BALL_NAME=red_ball_1
export BALL_X=1.50
export BALL_Y=0.50
export BALL_Z=0.10
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
gz service --list | rg '/world/default/create'
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'sdf: "<sdf version=\"1.9\"><model name=\"red_ball\"><static>true</static><link name=\"link\"><collision name=\"collision\"><geometry><sphere><radius>0.10</radius></sphere></geometry></collision><visual name=\"visual\"><geometry><sphere><radius>0.10</radius></sphere></geometry><material><diffuse>0.9 0.05 0.05 1</diffuse><ambient>0.9 0.05 0.05 1</ambient></material></visual></link></model></sdf>" pose { position { x: '"$BALL_X"' y: '"$BALL_Y"' z: '"$BALL_Z"' } } name: '"$BALL_NAME"' allow_renaming: false'
```

运行时并不读取 Gazebo 的 instance-segmentation 标签，因此手动球不需要 Oracle Label 插件；它是否可接近只由实际 RGB 图像、YOLO 15 帧确认、语言颜色和 Nav2 决定。若创建服务不存在，先停止并检查 Gazebo 是否仍在运行，不要改为直接发布底盘速度。

### 8.3 启动 Nav2 与策略服务

Gazebo 保持运行后，在第二个终端启动定位和 Nav2：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi_cam
ros2 launch turtlebot3_navigation2 my_nav2.launch.py use_sim_time:=true
```

在第三个终端启动仅监听回环地址的策略服务。固定 seed 使同一输入产生可复现的推理噪声；不要把服务暴露到局域网：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export CONDA_ROOT=$HOME/miniconda3
export VLA_CONDA_ENV=lerobot
export VLA_CHECKPOINT="$TB3_WS/outputs/smolvla_formal_20260728/checkpoints/010000/pretrained_model"
export VLA_BASE_MODEL_DIR="$TB3_WS/artifacts/smolvlm2-500m-video-instruct"
cd "$TB3_WS"
"$CONDA_ROOT/bin/conda" run -n "$VLA_CONDA_ENV" python tools/vla/vla_policy_server.py \
  --checkpoint "$VLA_CHECKPOINT" \
  --vlm-model-dir "$VLA_BASE_MODEL_DIR" \
  --device cuda \
  --seed 20260727 \
  --model-version tb3-ball-vla-v1
```

### 8.4 Shadow：先看决策，不让 VLA 驱动机器人

在第四个终端启动感知和找物服务。Shadow 仍使用固定搜索运动，只记录 VLA 建议，适合观察“初始看不见球时是否选择候选点和扫描、看见球后是否建议对应颜色的接近”：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export YOLO11_ONNX="$TB3_WS/models/yolo11n_ball.onnx"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
sou
ros2 launch turtlebot3_embodied_navigation embodied_navigation.launch.py \
  model_path:="$YOLO11_ONNX" \
  use_sim_time:=true \
  policy_mode:=shadow \
  policy_endpoint:=http://127.0.0.1:8089/select_action
```

在第五个终端发送测试任务，并观察 Gazebo、RViz、Action feedback 和第四个终端的 `VLA shadow action` 日志：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 action send_goal \
  /find_object \
  turtlebot3_embodied_interfaces/action/FindObject \
  "{instruction: '找红球'}" \
  --feedback
```

Shadow 通过条件是：策略服务无超时；每个建议满足扫描阶段约束；`approach_red` 只在语言契约为红色时出现；原型距离或间隔不足的建议被安全门拦截；YOLO 与 Nav2 不可用时没有绕过安全层的运动。

### 8.5 Active：VLA 选择高层技能

只有完成完整 validation/test 门限校准、审阅 Shadow 日志并确认没有错误接近动作后，停止 Shadow 进程并以 Active 重启第四个终端。不要用少量抽样的原型距离/间隔直接改写门限。

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export YOLO11_ONNX="$TB3_WS/models/yolo11n_ball.onnx"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_embodied_navigation embodied_navigation.launch.py \
  model_path:="$YOLO11_ONNX" \
  use_sim_time:=true \
  policy_mode:=active \
  policy_endpoint:=http://127.0.0.1:8089/select_action
```

随后复用上一节的 `/find_object` Action 命令。Active 没有固定策略回退，策略服务、模型版本、原型门限、相机时效、YOLO 确认或 Nav2 任一异常都会使任务失败关闭，而不是猜测执行。

## 9. 连续局部目标 v2：LLM 分阶段规划，VLA 生成局部目标

本节是新的训练和运行通道，不复用 `formal_20260728` 等离散 one-hot 数据。旧模型和旧数据继续作为 `discrete_skill` 回退基线。

v2 的职责边界如下：

- LLM 先解析目标类别/颜色；之后只在阶段完成时选择 `search`、`approach`、`verify`、`done` 或 `fail`，不能给坐标或速度；
- SmolVLA 输入 RGB、LLM 的规范化阶段指令和 25 维状态，回归 `[delta_x_m, delta_y_m, delta_yaw_rad]`；
- 输出必须满足平移不超过 `0.75 m`、转角不超过 `π/4`，零进展输出会被拒绝；
- 节点把相对目标转换到 `map`，只交给 Nav2；它绝不发布 `/cmd_vel`；
- 接近仍要求 YOLO 最近 15 帧至少 7 帧命中、检测新鲜、里程计存在且 Nav2 成功。任一失败都会停止任务。

因此 SmolVLA 默认的连续回归损失与 v2 动作天然匹配；不为离散 one-hot 动作修改 LeRobot 的损失函数。

### 9.1 新数据的 Diagnostic 与 Pilot 门禁

连续 Oracle 会保存 25 维状态、3 维有界局部目标、阶段指令、Oracle 目标可见性和真实位姿审计字段。目标不可见时监督 `search` 到最近未访问视点的局部目标；目标可见时监督保持 `0.8 m` 距离的 `approach` 局部目标。Oracle 真值不会进入模型状态。

先运行 10 Episode Diagnostic；目录必须是全新的。Diagnostic 只用于排障，不能转换或训练：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_diagnostic_20260730"
cd "$TB3_WS"
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

生成完成后立即验证。命令返回非零时，不得进入 Pilot：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_diagnostic_20260730"
export VLA_REPORT_DIR="$TB3_WS/datasets/vla_reports/continuous_diagnostic_20260730"
cd "$TB3_WS"
python3 tools/vla/validate_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" \
  --report-dir "$VLA_REPORT_DIR"
```

Diagnostic 通过后运行 200 Episode Pilot，并以人工审阅 `report.md` 为门禁；Pilot 也不训练：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_pilot_20260730"
cd "$TB3_WS"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_gazebo vla_oracle_dataset.launch.py \
  output_dir:="$VLA_RAW_DIR" \
  policy_interface:=continuous_local_goal \
  episode_count:=200 \
  target_absent_ratio:=0.30 \
  seed:=20260730 \
  preview:=false
```

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_pilot_20260730"
export VLA_REPORT_DIR="$TB3_WS/datasets/vla_reports/continuous_pilot_20260730"
cd "$TB3_WS"
python3 tools/vla/validate_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" \
  --report-dir "$VLA_REPORT_DIR"
```

人工门禁必须确认：报告通过；目标存在 Episode 的最后一帧是可见的 `approach`；目标不存在 Episode 没有 `approach` 且标为完整覆盖；所有动作保持在局部目标边界内。

### 9.2 正式 2000 Episode、验证和转换

Pilot 通过并完成上述人工门禁后，才允许创建批准文件。中断恢复必须使用完全相同的目录、seed、总 Episode 数和 `policy_interface`：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_formal_20260730"
export VLA_APPROVAL_FILE="$TB3_WS/datasets/vla_raw/continuous_formal_approval_20260730.txt"
cd "$TB3_WS"
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

正式集验证、人工审阅后才转换。转换器会核对验证报告中的原始清单摘要，防止验证后悄悄改标注：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export CONDA_ROOT=$HOME/miniconda3
export VLA_CONDA_ENV=lerobot
export VLA_RAW_DIR="$TB3_WS/datasets/vla_raw/continuous_formal_20260730"
export VLA_REPORT_DIR="$TB3_WS/datasets/vla_reports/continuous_formal_20260730"
export VLA_LE_ROBOT_ROOT="$TB3_WS/datasets/vla_lerobot/continuous_formal_20260730"
cd "$TB3_WS"
python3 tools/vla/validate_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" \
  --report-dir "$VLA_REPORT_DIR"
"$CONDA_ROOT/bin/conda" run -n "$VLA_CONDA_ENV" python tools/vla/convert_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" --validation-report "$VLA_REPORT_DIR/report.json" --split train \
  --output-root "$VLA_LE_ROBOT_ROOT/train" --repo-id local/tb3_ball_continuous_vla_train
"$CONDA_ROOT/bin/conda" run -n "$VLA_CONDA_ENV" python tools/vla/convert_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" --validation-report "$VLA_REPORT_DIR/report.json" --split validation \
  --output-root "$VLA_LE_ROBOT_ROOT/validation" --repo-id local/tb3_ball_continuous_vla_validation
"$CONDA_ROOT/bin/conda" run -n "$VLA_CONDA_ENV" python tools/vla/convert_continuous_vla_dataset.py \
  --raw-root "$VLA_RAW_DIR" --validation-report "$VLA_REPORT_DIR/report.json" --split test \
  --output-root "$VLA_LE_ROBOT_ROOT/test" --repo-id local/tb3_ball_continuous_vla_test
```

### 9.3 训练、Shadow 与 Active 门禁

连续模型不使用离散动作均衡器。训练只使用 `train/`；validation/test 保持为评估集：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export CONDA_ROOT=$HOME/miniconda3
export VLA_CONDA_ENV=lerobot
export VLA_CONTINUOUS_TRAIN_ROOT="$TB3_WS/datasets/vla_lerobot/continuous_formal_20260730/train"
export VLA_CONTINUOUS_OUTPUT_DIR="$TB3_WS/outputs/smolvla_continuous_formal_20260730"
export VLA_CONTINUOUS_TRAIN_REPO_ID=local/tb3_ball_continuous_vla_train
cd "$TB3_WS"
"$CONDA_ROOT/bin/conda" run -n "$VLA_CONDA_ENV" bash tools/vla/train_continuous_smolvla.sh
```

先以 Shadow 运行。它仍按固定视点搜索，只记录连续局部目标，且要求 `/odom`、相机与策略服务都新鲜：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export CONDA_ROOT=$HOME/miniconda3
export VLA_CONDA_ENV=lerobot
export VLA_CONTINUOUS_CHECKPOINT="$TB3_WS/outputs/smolvla_continuous_formal_20260730/checkpoints/010000/pretrained_model"
export VLA_BASE_MODEL_DIR="$TB3_WS/artifacts/smolvlm2-500m-video-instruct"
export YOLO11_ONNX="$TB3_WS/models/yolo11n_ball.onnx"
cd "$TB3_WS"
"$CONDA_ROOT/bin/conda" run -n "$VLA_CONDA_ENV" python tools/vla/vla_policy_server.py \
  --checkpoint "$VLA_CONTINUOUS_CHECKPOINT" \
  --vlm-model-dir "$VLA_BASE_MODEL_DIR" \
  --interface continuous_local_goal \
  --device cuda --seed 20260730 --model-version tb3-ball-continuous-v2
```

另开终端启动连续 Shadow：

```bash
export TB3_WS=$HOME/GitHub/tb3_jazzy_ws
export YOLO11_ONNX="$TB3_WS/models/yolo11n_ball.onnx"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
ros2 launch turtlebot3_embodied_navigation embodied_navigation.launch.py \
  model_path:="$YOLO11_ONNX" use_sim_time:=true policy_mode:=shadow \
  policy_interface:=continuous_local_goal \
  continuous_policy_endpoint:=http://127.0.0.1:8089/predict_local_goal
```

只有 Shadow 日志确认局部目标全在边界内、接近前的 YOLO 7/15 确认有效、没有绕过 Nav2 后，才将上面命令的 `policy_mode:=shadow` 改为 `policy_mode:=active`。Active 验收为 `turtlebot3_house` 中 10 次任务至少 8 次成功、每次最终停在球 `0.8 m` 范围、零碰撞；未达到门槛时保留离散基线，不替换默认运行模式。

最终验收使用未参与训练的 `turtlebot3_house` 位置。每次记录指令、球颜色、起始位姿、模型名与位姿、访问视点、策略动作、原型距离/间隔、15 帧检测证据、Nav2 结果和最终距离。合格条件是：不误接近颜色不符目标、停在目标 `0.8 m` 范围内、失败时安全停止且不发送直接底盘速度。

## 9. 故障定位顺序

1. 策略服务返回 4xx/5xx：核对 checkpoint、Conda 环境、23 维状态和 JPEG 上限；Active 应失败关闭。
2. `prototype_distance/gap` 被拒：先检查验证集分布和模型输出，再重新校准门限，不要直接放宽阈值。
3. 接近动作被拒：检查语言契约、YOLO 最近 15 帧中的 7 帧确认与检测时间；这属于预期安全拦截。
4. Nav2 失败：按现有 Nav2 文档检查 `map -> odom -> base_link`、costmap 和可达性；不要让 VLA 绕过 Nav2。
