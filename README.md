# uuv_simulator —— 管道自主巡检仿真

基于 **ROS2 Humble + Gazebo** 的管道自主巡检水下机器人（BlueROV2）仿真系统，包含运动控制、Gazebo 场景、传感器仿真，以及基于 SVIn 的视觉惯性导航评估。

## 系统组成

完整系统由两个仓库、两个 colcon 工作区组成：

| 仓库 | 工作区 | 内容 |
|---|---|---|
| [uuv_simulator](https://github.com/Yang-boy666/uuv_simulator)（本仓库） | `~/uuv_sim_ws` | 三个自研包 project3_gazebo / project3_control / project3_sensors + 6 个第三方包（.repos 管理） |
| [svin_ws](https://github.com/Yang-boy666/svin_ws)（配套） | `~/svin_ws` | SVIn 视觉惯性导航（okvis_ros、pose_graph），提供 `okvis_ros` 给 project3_sensors 使用 |

## 环境要求

- Ubuntu 22.04
- ROS2 Humble（desktop 全量安装）
- Gazebo（`ros-humble-ros-gz`，与 Humble 配套）

## 从零复现

### 0. 基础工具

```bash
sudo apt update
sudo apt install -y python3-vcstool python3-rosdep python3-colcon-common-extensions
sudo rosdep init && rosdep update
```

### 1. 主工作区 `uuv_sim_ws`

```bash
git clone https://github.com/Yang-boy666/uuv_simulator.git ~/uuv_sim_ws
cd ~/uuv_sim_ws
vcs import src < uuv_sim.repos                          # 还原 6 个第三方包
rosdep install --from-paths src --ignore-src -r -y      # 装 robot_localization、ros_gz 等
```

### 2. SVIn 工作区 `svin_ws`（提供 okvis_ros）

```bash
# 2.1 系统依赖
sudo apt install -y libpcl-dev libgoogle-glog-dev libgflags-dev \
  libblas-dev liblapack-dev libatlas-base-dev libeigen3-dev \
  libsuitesparse-dev libopencv-dev
sudo apt install -y ros-humble-pcl-ros ros-humble-tf2-sensor-msgs \
  ros-humble-compressed-image-transport ros-humble-cv-bridge ros-humble-image-transport

# 2.2 BRISK 特征（源码安装，一次性）
cd ~ && wget https://www.doc.ic.ac.uk/~sleutene/software/brisk-2.0.8.zip && unzip brisk-2.0.8.zip
cd brisk && mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc) && sudo make install

# 2.3 SVIn 本体（自身就是完整工作区，直接 clone）
git clone https://github.com/Yang-boy666/svin_ws.git ~/svin_ws
cd ~/svin_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> SVIn 更详细的依赖说明见其仓库内 `src/SVIn/install.md`。

### 3. 编译主工作区

```bash
cd ~/uuv_sim_ws
source /opt/ros/humble/setup.bash
source ~/svin_ws/install/setup.bash      # 关键：先 source SVIn，提供 okvis_ros
colcon build --symlink-install
source install/setup.bash
```

## 运行

启动顺序、参数调优等见 [管道自主巡检仿真运行与调参说明.md](管道自主巡检仿真运行与调参说明.md)。

## 第三方依赖说明

- 本仓库只提交自研代码，6 个第三方 ROS 包（auv_control、bluerov2、pose_to_tf、simple_launch、slider_publisher、thruster_manager）由 `uuv_sim.repos` 锁定版本，通过 `vcs import` 还原。
- SVIn（okvis_ros 等）位于配套仓库 `svin_ws`，由 `git clone` 直接还原。
