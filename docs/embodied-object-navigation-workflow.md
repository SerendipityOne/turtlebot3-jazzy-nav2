# 具身找球运行手册

本手册只保留当前已验证的最小闭环：用户用自然语言要求寻找球体，系统搜索预设视点，使用单类 YOLO11n 模型识别 `ball`，再由 Nav2 将机器人接近至目标约 `0.8 m`。

## 已验证边界

- 机器人模型：`waffle_pi_cam`。
- 检测类别：仅 `ball`；墙面、桌面等背景不应作为任务目标。
- 运行时阈值：置信度 `0.47647647647647645`。
- 接近距离：`0.8 m`。
- 最小验收：将红球放在初始相机视野之外，任务最终返回 `success: true`、`target_class: ball` 和 `target found and verified`。

一次成功只证明系统闭环可用，不代表所有房间、光照和摆放位置下的成功率。

## 启动

先确认 `.env` 已由 `.env.example` 创建并填入有效的 `VLM_API_BASE`、`VLM_MODEL` 和 `VLM_API_KEY`。模型文件是本地工件，受 Git 忽略，不会随仓库克隆。

终端 1，启动 house 场景：

```bash
export TB3_WS=/home/yzh/GitHub/tb3_jazzy_ws
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi_cam

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

终端 2，启动 Nav2：

```bash
export TB3_WS=/home/yzh/GitHub/tb3_jazzy_ws
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi_cam

ros2 launch turtlebot3_navigation2 my_nav2.launch.py use_sim_time:=true
```

终端 3，启动感知与找物 Action：

```bash
export TB3_WS=/home/yzh/GitHub/tb3_jazzy_ws
export YOLO11_ONNX="$TB3_WS/artifacts/yolo11n_ball.onnx"
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi_cam
test -r "$YOLO11_ONNX"
set -a
source "$TB3_WS/.env"
set +a

ros2 launch turtlebot3_embodied_navigation embodied_navigation.launch.py \
  model_path:="$YOLO11_ONNX" \
  use_sim_time:=true
```

终端 4，发送任务并查看阶段反馈：

```bash
export TB3_WS=/home/yzh/GitHub/tb3_jazzy_ws
source /opt/ros/jazzy/setup.bash
source "$TB3_WS/install/setup.bash"

ros2 action send_goal \
  /find_object \
  turtlebot3_embodied_interfaces/action/FindObject \
  "{instruction: '找红球'}" \
  --feedback
```

正常过程依次经过 `parsing`、`searching_viewpoints`、`approaching` 和 `verifying`。若 Action 返回 `success: true`，说明目标已经通过多帧检测确认并完成接近；若在更换地图后失败，应重新配置 `find_object.yaml` 中的搜索视点。
