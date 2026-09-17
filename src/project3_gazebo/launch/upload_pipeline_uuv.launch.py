from simple_launch import GazeboBridge, SimpleLauncher


sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg("namespace", default_value="pipeline_uuv")
sl.declare_arg("ground_truth", default_value=True)
sl.declare_arg("sliders", default_value=False)
sl.declare_arg("gazebo_world_name", default_value="pipeline_pool")
sl.declare_gazebo_axes(x=-8.5, y=-5.0, z=-4.0, roll=0.0, pitch=0.0, yaw=0.0)


def launch_setup():
    ns = sl.arg("namespace")
    GazeboBridge.set_world_name(sl.arg("gazebo_world_name"))

    with sl.group(ns=ns):
        sl.robot_state_publisher(
            "project3_gazebo",
            "pipeline_bluerov2.xacro",
            xacro_args={"namespace": ns, "simulation": sl.sim_time},
        )
        sl.spawn_gz_model(ns, spawn_args=sl.gazebo_axes_args())

        bridges = [
            GazeboBridge(
                GazeboBridge.model_prefix(ns) + "/joint_state",
                "joint_states",
                "sensor_msgs/JointState",
                GazeboBridge.gz2ros,
            ),
            GazeboBridge(
                f"/model/{ns}/pose",
                "pose_gt",
                "geometry_msgs/Pose",
                GazeboBridge.gz2ros,
            ),
            GazeboBridge(
                f"/{ns}/sonar", "sonar", "sensor_msgs/LaserScan", GazeboBridge.gz2ros
            ),
            GazeboBridge(
                f"/{ns}/sonar/points",
                "cloud",
                "sensor_msgs/PointCloud2",
                GazeboBridge.gz2ros,
            ),
            GazeboBridge(
                f"/{ns}/image",
                "/camera/left/image_raw",
                "sensor_msgs/Image",
                GazeboBridge.gz2ros,
            ),
            GazeboBridge(
                f"/{ns}/image_right",
                "/camera/right/image_raw",
                "sensor_msgs/Image",
                GazeboBridge.gz2ros,
            ),
            GazeboBridge(
                f"/{ns}/svin_imu",
                "/imu/data_raw",
                "sensor_msgs/Imu",
                GazeboBridge.gz2ros,
            ),
            GazeboBridge(
                f"/model/{ns}/joint/tilt/0/cmd_pos",
                "cmd_tilt",
                "std_msgs/Float64",
                GazeboBridge.ros2gz,
            ),
        ]

        for beam_index in range(1, 5):
            bridges.append(
                GazeboBridge(
                    f"/{ns}/dvl/beam{beam_index}",
                    f"/dvl/beam{beam_index}",
                    "sensor_msgs/LaserScan",
                    GazeboBridge.gz2ros,
                )
            )

        if sl.arg("ground_truth"):
            bridges.append(
                GazeboBridge(
                    f"/model/{ns}/odometry",
                    "odom",
                    "nav_msgs/Odometry",
                    GazeboBridge.gz2ros,
                    "gz.msgs.Odometry",
                )
            )
            # Visualization-only world -> base transform. Navigation never
            # consumes this TF; it uses the simulated sensor topics instead.
            sl.node(
                "pose_to_tf",
                parameters={"child_frame": ns + "/base_link"},
                output="screen",
            )

        for thruster_index in range(1, 7):
            bridges.append(
                GazeboBridge(
                    f"/{ns}/thruster{thruster_index}/cmd",
                    f"cmd_thruster{thruster_index}",
                    "std_msgs/Float64",
                    GazeboBridge.ros2gz,
                )
            )

        sl.create_gz_bridge(bridges, name="pipeline_uuv_bridge")

        if sl.arg("sliders"):
            sl.node(
                "slider_publisher",
                arguments=[sl.find("bluerov2_description", "manual.yaml")],
            )

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
