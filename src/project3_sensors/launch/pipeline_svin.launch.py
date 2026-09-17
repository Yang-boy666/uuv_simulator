from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    autonomy = LaunchConfiguration("autonomy")
    rviz = LaunchConfiguration("rviz")
    enabled = LaunchConfiguration("enabled")
    feature_view = LaunchConfiguration("feature_view")
    fusion_view = LaunchConfiguration("fusion_view")
    reconstruction = LaunchConfiguration("reconstruction")

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("project3_gazebo"), "launch", "pipeline_pool.launch.py"]
    )
    navigation_launch = PathJoinSubstitution(
        [FindPackageShare("project3_sensors"), "launch", "svin_navigation.launch.py"]
    )
    control_launch = PathJoinSubstitution(
        [FindPackageShare("project3_control"), "launch", "pipeline_autonomy.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("autonomy", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("feature_view", default_value="true"),
            DeclareLaunchArgument("fusion_view", default_value="true"),
            DeclareLaunchArgument("reconstruction", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={"gui": gui, "namespace": "pipeline_uuv"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "enable_svin": "true",
                    "feature_view": feature_view,
                    "fusion_view": fusion_view,
                    "reconstruction": reconstruction,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(control_launch),
                condition=IfCondition(autonomy),
                launch_arguments={
                    "namespace": "pipeline_uuv",
                    "enabled": enabled,
                    "odom_topic": "/navigation/odom",
                    "sonar_topic": "sonar",
                    "rviz": rviz,
                }.items(),
            ),
        ]
    )
