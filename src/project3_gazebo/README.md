# project3_gazebo

Gazebo Fortress scene for internal pipeline inspection with the existing
`CentraleNantesROV/bluerov2` model.

## Scene geometry

- Pool inner size: 24 m x 18 m x 8 m.
- Water surface: z = 0 m; pool bottom: z = -8 m.
- Pipe center depth: 4 m.
- First-version pipe inner diameter: 1.2 m.
- Route: three straight sections and two horizontal 90 degree elbows.
- Default robot spawn: (-8.5, -5.0, -4.0), inside the first straight section.

The pipe is an octagonal hollow shell made from overlapping collision panels.
This is lighter and more stable in Gazebo Fortress than a high-resolution
triangle mesh while retaining internal wall collisions. The shell is partly
transparent in the GUI so that the robot can be inspected from outside.

## Regenerate and validate the world

```bash
cd ~/uuv_sim_ws
python3 src/project3_gazebo/scripts/generate_pipeline_world.py
ign sdf -k src/project3_gazebo/worlds/pipeline_pool.sdf
```

## Build and launch

```bash
cd ~/uuv_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select project3_gazebo
source install/setup.bash
ros2 launch project3_gazebo pipeline_pool.launch.py
```

Headless validation:

```bash
ros2 launch project3_gazebo pipeline_pool.launch.py gui:=false
```

If another BlueROV2 simulation is already running, use a separate model/topic
namespace to avoid mixing their Ground Truth and thruster topics:

```bash
ros2 launch project3_gazebo pipeline_pool.launch.py \
  gui:=false namespace:=pipeline_test
```

Manual thruster sliders can be enabled with `sliders:=true`. They are only for
actuator checks; pipe following will use a synchronized controller in a later
step.

The project launch also supplies the missing upstream IMU bridge and publishes
the actual Gazebo IMU as `/imu/data_raw`.

## Manual motion

Start with small values (1--3 N). The helper publishes all six channels at the
same time and always sends repeated zeros before exiting:

```bash
# Forward for 2 seconds
ros2 run project3_gazebo thruster_command.py \
  --namespace bluerov2 --surge 2.0 --duration 2.0

# Turn counter-clockwise
ros2 run project3_gazebo thruster_command.py \
  --namespace bluerov2 --yaw 1.0 --duration 1.0

# Force all channels to zero
ros2 run project3_gazebo thruster_command.py \
  --namespace bluerov2 --stop
```

Positive axes follow ROS ENU: surge forward, sway left, heave up and yaw
counter-clockwise. These are manual mixer commands, not the final calibrated
body-wrench allocator.

## Autonomous pipe following

The autonomous controller is split into guidance, velocity PI control and
thruster allocation nodes. Start it in a second terminal with the same Gazebo
partition and ROS domain as the simulation:

```bash
ros2 launch project3_control pipeline_autonomy.launch.py \
  namespace:=pipeline_uuv enabled:=true
```

See `project3_control/README.md` for the enable/stop service, diagnostics and
parameter tuning interface.
