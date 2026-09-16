# STEP 1：BlueROV2 / ROS 2 Humble / Gazebo Fortress 基线报告

执行日期：2026-09-16

## 1. 结论

STEP 1 已完成。`CentraleNantesROV/bluerov2` 及其依赖已经放入
`~/uuv_sim_ws/src`，7 个 ROS 2 包全部编译成功。Gazebo Fortress
无界面场景能够启动并生成 BlueROV2，ROS-Gazebo bridge 能建立 6 路推进器
指令接口，Ground Truth 里程计稳定发布，推进器指令能驱动车体运动。

当前未进入传感器适配、导航融合、规划和控制器开发。

## 2. 环境检查

| 项目 | 检查结果 |
|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS (Jammy) |
| 内核 | Linux 6.8.0-124-generic x86_64 |
| ROS 2 | Humble |
| Gazebo Sim | Fortress / Ignition Gazebo 6.16.0 |
| ros_gz | 0.244.23 |
| 编译器 | GCC/G++ 11.4.0 |
| 构建工具 | colcon 0.20.1 |

系统已安装 `ignition-fortress`、`ros-humble-ros-gz`、
`ros-humble-ros-gz-bridge`、`ros-humble-ros-gz-image` 和
`ros-humble-ros-gz-sim`，版本组合符合 Humble + Fortress 路线。

## 3. 工作空间结构

```text
~/uuv_sim_ws/
├── build/
├── install/
├── log/
├── STEP1_REPORT.md
├── uuv_sim.repos
└── src/
    ├── auv_control/
    ├── bluerov2/
    │   ├── bluerov2_control/
    │   └── bluerov2_description/
    ├── pose_to_tf/
    ├── simple_launch/
    ├── slider_publisher/
    └── thruster_manager/
```

`uuv_sim.repos` 固定了本次验证过的 6 个仓库提交，用于论文实验复现。

## 4. 依赖处理

上游 `bluerov2_description/package.xml` 没有完整声明运行依赖，因此仅运行
`rosdep` 不会自动发现 `simple_launch`、`pose_to_tf` 和
`slider_publisher`。依赖依据上游 README、Dockerfile 和
`bluerov2_control/ros2.repos` 确定。

本机使用 `sudo apt install` 时需要交互密码，因此没有改动系统软件。缺少的
`simple_launch` 和 `slider_publisher` 改为源码构建；其他上游依赖也固定为
源码提交。没有修改任何第三方源码。

从空目录复现源码的推荐命令：

```bash
mkdir -p ~/uuv_sim_ws/src
cd ~/uuv_sim_ws
vcs import src < uuv_sim.repos

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

本次 `rosdep` 结果：

```text
#All required rosdeps installed successfully
```

## 5. 编译与测试结果

构建命令：

```bash
cd ~/uuv_sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --event-handlers console_direct+
```

结果：

```text
Summary: 7 packages finished [1min 1s]
```

成功包：

```text
auv_control
bluerov2_control
bluerov2_description
pose_to_tf
simple_launch
slider_publisher
thruster_manager
```

测试命令：

```bash
source /opt/ros/humble/setup.bash
source ~/uuv_sim_ws/install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

结果：7 个包完成测试阶段，但上游当前没有注册自动测试用例：

```text
Summary: 0 tests, 0 errors, 0 failures, 0 skipped
```

## 6. 启动方法与结果

有桌面环境时按上游默认方式启动：

```bash
cd ~/uuv_sim_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bluerov2_description world_launch.py
```

服务器或调试时使用无界面模式：

```bash
ros2 launch bluerov2_description world_launch.py gui:=false spawn:=true
```

运行结果：

- Gazebo world `ocean` 加载成功。
- BlueROV2 entity 创建成功。
- `/clock`、Ground Truth、相机、声呐和推进器 bridge 均被创建。
- `/bluerov2/odom` 实测约 19.7--19.9 Hz，目标配置为 20 Hz。
- `world -> bluerov2/base_link` 动态 TF 可查询。
- `robot_state_publisher` 发布相机、IMU、声呐和 6 个推进器的子链接。

## 7. 当前 ROS 2 接口

主要可用接口如下：

| Topic | 类型 | 方向/用途 |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> ROS 2 |
| `/bluerov2/odom` | `nav_msgs/msg/Odometry` | Ground Truth，约 20 Hz |
| `/bluerov2/pose_gt` | `geometry_msgs/msg/Pose` | Ground Truth pose |
| `/bluerov2/image` | `sensor_msgs/msg/Image` | 单目图像 |
| `/bluerov2/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| `/bluerov2/sonar` | `sensor_msgs/msg/LaserScan` | 当前声呐扫描 |
| `/bluerov2/cloud` | `sensor_msgs/msg/PointCloud2` | 当前声呐点云 |
| `/bluerov2/cmd_thruster1..6` | `std_msgs/msg/Float64` | ROS 2 -> Gazebo，单位 N |
| `/bluerov2/cmd_tilt` | `std_msgs/msg/Float64` | 相机俯仰关节 |

基础包当前使用 `world -> bluerov2/base_link`。任务书要求的
`world -> map -> odom -> base_link` 将在后续 `project3_*` 扩展包中建立，
不直接破坏第三方模型的 TF。

## 8. 推进器验证

对 1--4 号水平推进器并行发送 `15 N`，持续约 3 秒后全部归零。测试前后
Ground Truth 位置为：

```text
before: x=0.99996, y=0.00001, z=-0.02519
after:  x=5.02867, y=0.62565, z=-0.03141
```

位移约 `4.08 m`，证明 ROS 2 指令经过 `ros_gz_bridge` 到达 Gazebo
推进器插件，水动力模型响应正常。测试后 1--4 号指令均已归零。

注意：该组同值指令伴随明显偏航，不能作为最终纯 surge 分配方案。STEP 2
需要用持续、同步发布器完成逐桨测试，确认每个推进器的正方向、力矩方向、
饱和值和最终分配矩阵。

单路手动命令示例：

```bash
ros2 topic pub --once /bluerov2/cmd_thruster1 \
  std_msgs/msg/Float64 "{data: 10.0}"

# 测试后必须归零
ros2 topic pub --once /bluerov2/cmd_thruster1 \
  std_msgs/msg/Float64 "{data: 0.0}"
```

## 9. 已发现问题

1. 当前上游 `main` 分支的 IMU 名称不一致：模型实际在 Gazebo 发布
   `/bluerov2/imu`，启动文件仍桥接旧的 `/bluerov2/mpu` 和
   `/bluerov2/lsm`。ROS 2 中能看到桥接端创建的话题名，但收不到模型 IMU
   数据。该问题不影响 STEP 1 的模型与推进器验证，应在 STEP 4 通过
   `project3_*` 启动/适配层修正，不直接修改上游包。
2. 默认生成位置为 `z=1`，模型落水后最终漂在水面附近（约 `z=-0.03`）。
   后续水下实验应把初始位置改为 ENU 下的负深度，例如 `z=-2.0`，并验证
   悬浮稳定性。
3. KDL 提示根链接带惯量。Gazebo 动力学可正常运行，但后续若有依赖 KDL
   根惯量的算法，应在项目描述包中增加无惯量虚拟根链接。
4. 无界面软件渲染出现 `libEGL failed to create dri2 screen` 警告，但传感器、
   物理和 ROS bridge 继续运行；桌面 GPU 环境需要另做 GUI 验证。
5. 本次 `colcon test` 没有上游测试用例。后续 `project3_*` 包应增加 launch
   test 与 topic/TF 自动验收测试。

## 10. STEP 2 建议验收项

下一步只处理基础模型和推进器，不提前进入完整传感器链路：

1. 新建项目扩展包，保持第三方 `bluerov2` 仓库不修改。
2. 将出生点设为水下 2--3 m，记录无指令状态 60 s 的深度、姿态漂移。
3. 对 6 个推进器逐个施加正/负阶跃，建立符号与六自由度响应表。
4. 使用单一同步测试节点生成 surge、sway、heave、yaw 指令组合。
5. 验证最大推力饱和，并形成后续 `thruster_allocator` 使用的分配矩阵。
