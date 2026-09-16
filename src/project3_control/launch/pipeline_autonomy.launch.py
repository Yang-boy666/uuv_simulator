from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    enabled = LaunchConfiguration("enabled")
    odom_topic = LaunchConfiguration("odom_topic")
    sonar_topic = LaunchConfiguration("sonar_topic")
    rviz = LaunchConfiguration("rviz")
    config = PathJoinSubstitution(
        [FindPackageShare("project3_control"), "config", "pipeline_control.yaml"]
    )

    common = {
        "namespace": namespace,
        "output": "screen",
        "parameters": [config],
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="pipeline_uuv"),
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument("sonar_topic", default_value="sonar"),
            DeclareLaunchArgument("rviz", default_value="false"),
            Node(
                package="project3_control",
                executable="pipe_guidance",
                name="pipe_guidance",
                parameters=[config, {"enabled": enabled}],
                namespace=namespace,
                output="screen",
                remappings=[("odom", odom_topic), ("sonar", sonar_topic)],
            ),
            Node(
                package="project3_control",
                executable="velocity_controller",
                name="velocity_controller",
                remappings=[("odom", odom_topic)],
                **common,
            ),
            Node(
                package="project3_control",
                executable="thruster_allocator",
                name="thruster_allocator",
                **common,
            ),
            Node(
                package="project3_control",
                executable="trajectory_publisher",
                name="trajectory_publisher",
                remappings=[("odom", odom_topic)],
                **common,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="pipeline_rviz",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("project3_control"), "rviz", "pipeline.rviz"]
                    ),
                ],
                condition=IfCondition(rviz),
                output="screen",
            ),
        ]
    )
