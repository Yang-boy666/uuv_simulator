# project3_control

Modular first-stage autonomous pipe following for the Project 3 BlueROV2.

Data flow:

```text
sonar + odom/depth -> pipe_guidance -> cmd_vel
cmd_vel + odom     -> velocity_controller -> cmd_wrench
cmd_wrench         -> thruster_allocator (f = pinv(B) tau) -> T1..T6
```

## Run

Start the Gazebo scene first. In a second terminal, use the same
`IGN_PARTITION` and `ROS_DOMAIN_ID`, then run:

```bash
source /opt/ros/humble/setup.bash
source ~/uuv_sim_ws/install/setup.bash

ros2 launch project3_control pipeline_autonomy.launch.py \
  namespace:=pipeline_uuv enabled:=true
```

For a safe start with zero thruster output, omit `enabled:=true`, inspect the
sensor topics, then enable it at runtime:

```bash
ros2 service call /pipeline_uuv/enable_pipe_following \
  std_srvs/srv/SetBool "{data: true}"
```

Stop without closing the nodes:

```bash
ros2 service call /pipeline_uuv/enable_pipe_following \
  std_srvs/srv/SetBool "{data: false}"
```

## Diagnostic data

`/pipeline_uuv/pipe_guidance/diagnostics` is a `Float64MultiArray` containing:

```text
[left_range, right_range, forward_range,
 center_error, depth_error, heading_error,
 surge_cmd, sway_cmd, heave_cmd, yaw_rate_cmd]
```

`/pipeline_uuv/velocity_controller/diagnostics` contains the four velocity
errors followed by `[Fx, Fy, Fz, Mz]`. The allocator publishes the six final
forces on `/pipeline_uuv/thruster_allocator/commands`.

All gains and safety limits are in `config/pipeline_control.yaml`. The current
outer loop uses Gazebo odometry for depth and velocity development. For
sim-to-real experiments, remap `odom` to `/navigation/odom`; that estimated
odometry should be produced from IMU, DVL and pressure depth rather than Gazebo
ground truth.

```bash
ros2 launch project3_control pipeline_autonomy.launch.py \
  namespace:=pipeline_uuv enabled:=true \
  odom_topic:=/navigation/odom sonar_topic:=/sonar/scan
```
