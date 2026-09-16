from simple_launch import GazeboBridge, SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    sl.declare_arg("gui", default_value=True)
    sl.declare_arg("spawn", default_value=True)
    sl.declare_arg("namespace", default_value="bluerov2")
    sl.declare_arg("sliders", default_value=False)
    sl.declare_arg("camera", default_value=True)
    sl.declare_gazebo_axes(
        x=-8.5,
        y=-5.0,
        z=-4.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    )

    world_file = sl.find("project3_gazebo", "pipeline_pool.sdf")
    with sl.group(if_arg="gui"):
        sl.gz_launch(world_file, "-r")
    with sl.group(unless_arg="gui"):
        sl.gz_launch(world_file, "-r -s")

    sl.create_gz_bridge(
        [
            GazeboBridge.clock(),
            GazeboBridge(
                "/ocean_current",
                "/current",
                "geometry_msgs/Vector3",
                GazeboBridge.ros2gz,
            ),
        ],
        name="pipeline_pool_bridge",
    )

    with sl.group(if_arg="spawn"):
        sl.include(
            "project3_gazebo",
            "upload_pipeline_uuv.launch.py",
            launch_arguments={
                "namespace": sl.arg("namespace"),
                "gazebo_world_name": "pipeline_pool",
                "sliders": sl.arg("sliders"),
                "x": sl.arg("x"),
                "y": sl.arg("y"),
                "z": sl.arg("z"),
                "roll": sl.arg("roll"),
                "pitch": sl.arg("pitch"),
                "yaw": sl.arg("yaw"),
            },
        )

    return sl.launch_description()
