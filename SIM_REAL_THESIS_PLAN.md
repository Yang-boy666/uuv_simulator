# 水下导航仿真—实物论文闭环方案

更新日期：2026-09-16

## 1. 论文应当证明什么

本项目的论文主线不应只写成“搭建了 Gazebo 仿真，并完成了实物实验”，而应证明：

1. 同一套导航算法可以接收仿真或实物传感器数据，不修改算法核心代码。
2. 仿真可以提供精确真值、可控噪声和可重复故障，用于定量分析算法误差与鲁棒性。
3. 水池实验验证标定、时间同步、安装外参、通信和计算平台等真实工程因素。
4. 长距离实物实验验证算法的连续性、长期漂移、失败恢复和实时性。
5. 仿真结论和实物结论在误差趋势上相互支撑，并对二者差异给出合理解释。

最终证据链为：

```text
仿真 Perfect：验证算法实现正确
        ↓
仿真 Normal/Degraded：分析噪声、掉线和弱纹理影响
        ↓
水池：验证实物接口、同步、外参和短程精度
        ↓
长距离实物：验证长期漂移、连续性和实时性
        ↓
同一评价工具输出统一指标、曲线和统计表
```

## 2. 当前实物工程基线

### 2.1 双目与 SVIn

当前 ZED 原始输入：

```text
/zed/zed_node/left/color/raw/image
/zed/zed_node/right/color/raw/image
```

`stereo_sync` 转换为：

```text
/cam0/image_raw
/cam1/image_raw
```

SVIn/OKVIS 当前主要输出：

```text
/okvis_odometry       nav_msgs/msg/Odometry
/okvis_path           nav_msgs/msg/Path
/keyframe_pose        nav_msgs/msg/Odometry
/svin_health          okvis_ros/msg/SvinHealth
```

位姿图和回环模块主要输出：

```text
/uber_odometry        nav_msgs/msg/Odometry
/uber_path            nav_msgs/msg/Path
/loop_closure_path    nav_msgs/msg/Path
/global_map           sensor_msgs/msg/PointCloud2
```

建议论文中的最终导航输出统一使用：

```text
/navigation/odom      nav_msgs/msg/Odometry
/navigation/path      nav_msgs/msg/Path
/navigation/health    项目健康状态
```

由一个 `navigation_output_adapter` 选择 `/okvis_odometry` 或
`/uber_odometry`，而不是让控制器直接依赖算法内部话题名。

### 2.2 DVL-A50

实物驱动输出：

```text
/dvl/data             dvl_msgs/msg/DVL
/dvl/position         dvl_msgs/msg/DVLDR
```

融合时应优先使用 `/dvl/data.velocity`，并检查：

```text
velocity_valid
covariance
fom
beams[].valid
altitude
```

已有静止测试结果为约 `12.5 Hz`，速度模长均值约 `0.0077 m/s`，最大值约
`0.022 m/s`，四波束均有效。这些统计量可以作为仿真 `normal` 模式的第一版
噪声参数依据。

`/dvl/position` 是 DVL 内部积分结果，不应作为论文真值，也不建议直接作为
融合位置量测。

### 2.3 mFOSN

当前 `/mfosn/ins_data` 是 JSON 字符串，包含：

```text
经纬高
北/东/天速度
姿态
角速度
加速度
GNSS 状态
DVL 速度与有效间隔
组合导航状态
```

它适合通过适配器拆分成标准 ROS 2 消息。需要特别区分：

- 原始角速度和加速度可以转换为 `/imu/data_raw`。
- mFOSN 已融合后的经纬高/姿态可作为对照轨迹。
- 如果 mFOSN 的融合结果被输入被测算法，就不能再把同一个结果称为独立真值。

### 2.4 深度计

现有节点输出 `/depth`，消息为 `geometry_msgs/msg/PointStamped`，并把毫米深度
放在 `point.x`。这个接口容易发生单位和语义错误，必须由适配器转换为：

```text
/depth/pressure       sensor_msgs/msg/FluidPressure
/depth/value          std_msgs/msg/Float64，单位 m，正值表示水深
```

导航算法内部统一使用 ENU：位置 `z` 向上，因此水深 `depth` 转换为局部位置时
通常有 `z = -depth`，不能把两种符号混用。

## 3. 仿真与实物的统一接口

| 数据 | 仿真原始输入 | 实物原始输入 | 算法统一接口 |
|---|---|---|---|
| 左图 | Gazebo camera | ZED left raw | `/camera/left/image_raw` |
| 右图 | Gazebo camera | ZED right raw | `/camera/right/image_raw` |
| IMU | Gazebo IMU | mFOSN/IMU 驱动 | `/imu/data_raw` |
| DVL | Ground Truth 派生模型 | `/dvl/data` | `/dvl/velocity`、`/dvl/valid` |
| 深度 | Gazebo z/压力 | 当前 `/depth` | `/depth/pressure`、`/depth/value` |
| 声呐 | Gazebo ray sensor | 750D 驱动 | `/sonar/points` |
| 导航结果 | 同一导航算法 | 同一导航算法 | `/navigation/odom` |
| 真值 | Gazebo | RTK/动捕/LBL/独立高精惯导 | `/reference/odom` |

仿真专用 `/ground_truth/odom` 只允许进入传感器仿真器和评价程序，禁止进入导航
融合算法。评价前可将它重映射为 `/reference/odom`。

建议启动参数只切换数据源：

```bash
ros2 launch project3_bringup navigation.launch.py source:=sim
ros2 launch project3_bringup navigation.launch.py source:=real
ros2 launch project3_bringup navigation.launch.py source:=bag
```

三种模式启动相同的 SVIn/融合/回环节点，仅启动不同的 sensor adapter。

## 4. 如何查看传感器和运动数据

### 4.1 基础健康检查

```bash
ros2 topic list -t
ros2 node list

ros2 topic hz /imu/data_raw
ros2 topic hz /dvl/velocity
ros2 topic hz /navigation/odom

ros2 topic echo /imu/data_raw --once
ros2 topic echo /dvl/velocity --once
ros2 topic echo /dvl/valid --once
ros2 topic echo /depth/value --once
ros2 topic echo /navigation/odom --once
```

DVL 实物原始话题是 Best Effort QoS 时使用：

```bash
ros2 topic echo /dvl/data --qos-reliability best_effort
ros2 topic hz /dvl/data --qos-reliability best_effort
```

查看带宽与发布者：

```bash
ros2 topic bw /camera/left/image_raw
ros2 topic info /navigation/odom --verbose
```

### 4.2 图像、点云与轨迹

```bash
ros2 run rqt_image_view rqt_image_view
rviz2
```

RViz Fixed Frame 统一设为 `map`，显示：

- `Path`：`/navigation/path`
- `Path`：`/reference/path`
- `Odometry`：`/navigation/odom`
- `PointCloud2`：`/sonar/points`
- `PointCloud2`：`/global_map`
- `TF`
- `RobotModel`

### 4.3 TF 检查

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo base_link dvl_link
ros2 run tf2_tools view_frames
```

应只有一个节点发布每一条 TF，不能同时让 mFOSN、SVIn 和适配器重复发布
`map -> base_link`。

### 4.4 当前基础 BlueROV2 场景

在尚未完成统一适配器前，可以直接检查：

```bash
ign topic -l
ros2 topic echo /bluerov2/odom --once
ros2 topic hz /bluerov2/odom
ros2 topic echo /bluerov2/sonar --once
```

当前上游 IMU bridge 名称与模型不一致，不能把能看到 `/bluerov2/mpu` 话题名
误判为已有有效 IMU 数据；必须以 `ros2 topic hz` 和实际消息为准。

## 5. 运动与数据记录流程

每次实验都记录原始传感器、算法输出、真值、TF 和参数版本：

```bash
ros2 bag record -o run_001 \
  /tf /tf_static \
  /camera/left/image_raw /camera/right/image_raw \
  /camera/left/camera_info /camera/right/camera_info \
  /imu/data_raw /dvl/velocity /dvl/valid \
  /depth/value /sonar/points \
  /navigation/odom /navigation/path /navigation/health \
  /ground_truth/odom /reference/odom \
  /cmd_vel
```

实物实验没有 `/ground_truth/odom` 时不要制造“真值”；只记录真实存在的
`/reference/odom`，并在论文中说明参考系统及其精度。

每个 rosbag 配套保存一个 `run_manifest.yaml`：

```yaml
run_id: real_pool_normal_001
source: real
scene: pool
algorithm_commit: <git commit>
config_file: <config yaml>
calibration_file: <camera-imu calibration>
sensor_profile: normal
reference_source: motion_capture
operator: <name>
notes: <experiment notes>
```

这样可以避免后期无法确认某条轨迹使用了哪套参数。

## 6. 轨迹评价必须遵守的步骤

### 6.1 时间同步

先比较消息时间戳，而不是先画轨迹。应统计：

- 相机左右目时间差。
- 相机与 IMU 时间差。
- DVL、深度与 IMU 时间差。
- 导航输出相对传感器数据的延迟。

仿真全部使用 `/clock`。实物建议硬件同步；无法硬同步时记录时间偏置，并在
评价程序中估计一个固定时间偏移。不能为了得到更好结果随意逐段移动时间轴。

### 6.2 坐标和外参

统一使用 REP-103 ENU：`x` 前、`y` 左、`z` 上。评价前必须确认：

- mFOSN 的北东天/NED 到 ENU 转换。
- DVL 速度从 `dvl_link` 到 `base_link` 的旋转。
- 相机/IMU/DVL/深度计的固定外参。
- 参考系统测量点到 `base_link` 的杆臂补偿。

### 6.3 轨迹对齐

双目+IMU 是有尺度系统，主结果应采用刚体 `SE(3)` 对齐。不要把 `Sim(3)`
缩放后的结果作为唯一结果，否则会掩盖尺度漂移。可以把 Sim(3) 作为补充，专门
分析尺度误差。

### 6.4 定位指标

至少报告：

| 指标 | 意义 |
|---|---|
| ATE RMSE/mean/median/max | 全局位置误差 |
| RPE translation | 局部相对位移误差 |
| RPE rotation | 局部姿态误差 |
| X/Y/Z RMSE | 分轴误差，尤其关注深度方向 |
| roll/pitch/yaw RMSE | 姿态误差 |
| drift % | 终点或累计漂移/真实路程 |
| loop closure error | 回到起点时的闭合误差 |
| completeness | 有效导航输出时长/总时长 |
| failures/relocalizations | 跟踪丢失和恢复次数 |
| latency/CPU/RAM | 实时性和工程可部署性 |

RPE 应同时给出固定时间间隔和固定距离间隔，例如：

```text
1 s、10 s、60 s
10 m、50 m、100 m、500 m
```

长达 2 km 的实验不能只给一张轨迹图和终点误差。

## 7. “跑了 2 km”怎样变成论文证据

2 km 输出轨迹只证明节点连续工作，并不证明轨迹正确。论文至少需要下列一种
独立参考：

1. 陆地/水面：RTK-GNSS 或未参与融合的高精度惯导。
2. 水池：光学动捕、AprilTag 全局定位或全站仪。
3. 水下大场景：LBL/USBL、声学基站或可靠的稀疏控制点。
4. 无连续真值：起终点已知、多个测量控制点、闭环返回误差和分段里程约束。

若 mFOSN 只作为独立参考而未进入 SVIn，可以将其转换为 ENU 参考轨迹；如果
mFOSN/DVL 已参与被测融合，则必须另找参考系统，避免“用输入验证输出”。

推荐把 2 km 路线分段评价：

```text
0--500 m
500--1000 m
1000--1500 m
1500--2000 m
```

每段计算 ATE/RPE、跟踪完整率、特征点数、DVL 有效率和计算延迟，才能说明误差
如何随航程增长。

当前配置引用的 2026-07-20 bag 位于外接盘：

```text
/media/zhuchao/新加卷/zy/202607201915/rosbag2_2026_07_20-18_20_20
```

本次检查时该盘未挂载，因此尚未对所谓 2 km 原始 bag 做正式统计。挂载后应先
检查 bag topic、时长、时间戳、参考轨迹和实际路线长度，再生成任何论文结论。

本地已有 SVIn 结果中，部分约 60--88 m 轨迹具有约 3 m 的首尾差；另一些结果
出现数千米跳变或 NaN。超长估计路径可能是发散，而不是真实航程，因此评价程序
必须先做非有限值、速度突变和时间倒退检查。

## 8. 论文实验矩阵

### 8.1 仿真重复实验

每组至少运行 10 个随机种子，推荐 20 个，报告中位数、四分位数和 95% 置信区间。

| 场景 | Perfect | Normal | Degraded |
|---|---:|---:|---:|
| 空水域直线/方形/8 字 | 是 | 是 | 是 |
| 水池障碍场景 | 是 | 是 | 是 |
| 管道弱纹理场景 | 是 | 是 | 是 |

退化变量不要一次全部改变，应先做单因素，再做组合退化：

- DVL dropout：0%、10%、30%、60%。
- IMU bias/noise：1 倍、2 倍、5 倍。
- 图像：亮度下降、模糊、对比度下降、部分遮挡。
- 深度：偏置、缓慢漂移、短时中断。
- 声呐：点云稀疏和随机丢帧。

### 8.2 算法消融

建议比较：

```text
A：Stereo + IMU
B：Stereo + IMU + DVL
C：Stereo + IMU + DVL + Depth
D：Stereo + IMU + DVL + Depth + Loop Closure
```

所有方法使用同一 bag、同一初值、同一评价程序。主要观察 ATE、RPE、Z 轴误差、
完整率和运行时间。

### 8.3 实物实验

```text
静态 10 min：IMU bias、DVL 零速、深度漂移
水池短程：直线、方形、8 字、定深运动
水池退化：遮挡相机、抬高 DVL 失锁、弱光
长距离：500 m、1 km、2 km，至少 3 次重复
```

长距离实验应包括起终点照片/测量坐标、外部参考记录和传感器健康状态，不应只
保存算法输出轨迹。

## 9. 仿真与实物如何形成闭环论述

论文结果应按下面逻辑组织：

1. 用实物静态数据估计噪声、偏置、频率和掉线分布。
2. 将这些统计量写入仿真 `normal` 参数，而不是凭经验随意设噪声。
3. 在仿真中用真值验证误差随噪声/掉线变化的规律。
4. 在水池和长距离实物实验中验证相同规律是否出现。
5. 分析绝对数值差异来自光照、水体散射、安装振动、时延和模型误差。
6. 将实物观测到的新故障模式回填仿真 `degraded` 模式，再验证改进算法。

真正的闭环是“实物标定仿真参数—仿真指导算法改进—实物再次验证”，而不是两组
互不相关的演示。

## 10. 建议的后续实施顺序

1. 完成 STEP 2 推进器逐桨标定和水下悬停。
2. 建立 `project3_sensors` 统一适配器和 topic contract。
3. 先接 Ground Truth、IMU、Depth、DVL，每完成一个就做频率/单位/TF 测试。
4. 将仿真双目 remap 到 SVIn，保持 SVIn 核心代码不变。
5. 建立 `/navigation/odom` 输出适配器和 `/reference/odom` 评价接口。
6. 实现 rosbag manifest、轨迹导出和数据质量检查。
7. 先用一条 60--100 m 闭环打通 ATE/RPE，再处理 2 km 数据。
8. 最后进行多随机种子仿真、消融、水池和长距离实物实验。

现阶段最关键的不是立刻做复杂声呐或规划，而是先把统一接口、时间同步、TF、独立
真值和评价工具打通。只要这五项严格一致，后续增加 DVL、深度、声呐或改进算法时，
论文实验体系不需要推倒重来。
