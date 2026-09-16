from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_svin = LaunchConfiguration("enable_svin")
    sensor_config = PathJoinSubstitution(
        [FindPackageShare("project3_sensors"), "config", "sensors.yaml"]
    )
    svin_config = PathJoinSubstitution(
        [FindPackageShare("project3_sensors"), "config", "svin_pipeline_sim.yaml"]
    )
    ekf_config = PathJoinSubstitution(
        [FindPackageShare("project3_sensors"), "config", "ekf_svin_dvl_depth.yaml"]
    )

    stereo_sync = Node(
        package="okvis_ros",
        executable="stereo_sync",
        name="stereo_sync",
        output="screen",
        condition=IfCondition(enable_svin),
        parameters=[
            {
                "use_sim_time": True,
                "config_filename": svin_config,
                "left_img_topic": "/camera/left/image_raw",
                "right_img_topic": "/camera/right/image_raw",
                "compressed": False,
            }
        ],
    )
    okvis = Node(
        package="okvis_ros",
        executable="okvis_node",
        name="okvis_node",
        output="screen",
        condition=IfCondition(enable_svin),
        parameters=[
            {
                "use_sim_time": True,
                "config_filename": svin_config,
                "mesh_file": "firefly.dae",
            }
        ],
        remappings=[
            ("camera0", "/cam0/image_raw"),
            ("camera1", "/cam1/image_raw"),
            ("imu", "/imu/data_raw"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_svin", default_value="true"),
            SetEnvironmentVariable("ROS_LOG_DIR", "/tmp/ros_logs_pipeline_svin"),
            Node(
                package="project3_sensors",
                executable="sensor_simulator",
                name="sensor_simulator",
                parameters=[sensor_config],
                output="screen",
            ),
            Node(
                package="project3_sensors",
                executable="camera_info_publisher",
                name="camera_info_publisher",
                parameters=[sensor_config],
                output="screen",
            ),
            Node(
                package="project3_sensors",
                executable="dead_reckoning_guard",
                name="dead_reckoning_guard",
                parameters=[sensor_config],
                output="screen",
            ),
            # Let Gazebo publish clock and initial sensor samples before SVIn starts.
            TimerAction(period=2.0, actions=[stereo_sync, okvis]),
            Node(
                package="project3_sensors",
                executable="svin_odom_adapter",
                name="svin_odom_adapter",
                parameters=[sensor_config],
                condition=IfCondition(enable_svin),
                output="screen",
            ),
            # Starting the EKF after SVIn avoids initializing world X/Y from a
            # depth-only sample before the first visual-inertial pose arrives.
            TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package="robot_localization",
                        executable="ekf_node",
                        name="ekf_filter_node",
                        parameters=[ekf_config],
                        remappings=[("odometry/filtered", "/navigation/odom")],
                        condition=IfCondition(enable_svin),
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="project3_sensors",
                        executable="navigation_evaluator",
                        name="navigation_evaluator",
                        parameters=[sensor_config],
                        condition=IfCondition(enable_svin),
                        output="screen",
                    )
                ],
            ),
        ]
    )
