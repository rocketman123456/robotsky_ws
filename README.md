# robotsky_ws
workspace for robotsky wheel quadruped robot

most codes are generate by deepseek and chatgpt, with bug fix and hand-write low-level motor driver

```bash
ros2 topic pub --once /pause_flag std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /pause_flag std_msgs/msg/Bool "{data: false}"

ros2 run robotsky_sim robot_sim

ros2 launch robotsky_teleop xbox_teleop.launch.py

ros2 run joy joy_node --ros-args -p dev:=/dev/input/js1

ros2 run robotsky_teleop xbox_cmd_vel --ros-args \
  -p linear_scale:=1.0 \
  -p angular_scale:=1.5 \
  -p deadzone:=0.1

ros2 launch robotsky_motor hardware.launch.py

ros2 run robotsky_rl_controller rl_controller --ros-args \
  -p control_frequency:=50.0 \
  -p cpu_core:=2 \
  -p action_scale:=0.25 \
  -p wheel_action_scale:=4.0 \
  -p wheel_joint_indices:="[3,7,11,15]"


ros2 run robotsky_rl_controller rl_controller \
  --ros-args \
  -p model_path:=/home/robotsky/robotsky_ws/robotsky_ws/src/robotsky_rl_controller/model/2026-03-19_17-16-40/exported/exported_policy_robotsky_wq.pt \
  -p cpu_core:=2 \
  -p action_scale:=0.25 \
  -p wheel_action_scale:=4.0 \
  -p wheel_joint_indices:="[3,7,11,15]"

ros2 run robotsky_rl_controller rl_controller \
  --ros-args \
  -p model_path:=model/exported_policy_robotsky_wq-2026-03-19_16-07-26.onnx \
  -p cpu_core:=2 \
  -p action_scale:=0.25 \
  -p wheel_action_scale:=4.0 \
  -p wheel_joint_indices:="[3,7,11,15]"

# ros2 run robotsky_rl_controller rl_controller_node --ros-args \
#   -p standup_ramp_enable:=true \
#   -p standup_duration_s:=2.0 \
#   -p standup_settle_s:=1.0 \
#   -p standup_start_from_current:=true \
#   -p pause_initial:=false

# ros2 run robotsky_rl_controller rl_control
# ler \
#   --ros-args \
#   -p model_path:=model/exported_policy_robotsky_wq-2026-03-19_16-07-26.pt \
#   -p cpu_core:=2 \
#   -p standup_ramp_enable:=true \
#   -p standup_duration_s:=2.0 \
#   -p standup_settle_s:=1.0 \
#   -p standup_start_from_current:=true \
#   -p pause_initial:=false

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}" --once
```

on robot

```bash
sudo systemctl start hardware.service


```

hardware.service

```
[Unit]
Description=My Startup Script
After=network.target

[Service]
ExecStart=/home/robotsky/robotsky_ws/start_ethercat.sh
Type=simple

[Install]
WantedBy=multi-user.target

```