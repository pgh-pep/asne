import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    asne_directory = get_package_share_directory("asne_pkg")
    vrx_gz_directory = get_package_share_directory("vrx_gz")

    # GAZEBO SIMULATION
    vrx_worlds_directory = os.path.join(vrx_gz_directory, "worlds")

    resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = f"{resource_path}:{vrx_worlds_directory}"

    model_path = os.path.join(asne_directory, "urdf", "wamv_target.urdf")
    world = "sydney_regatta"

    vrx_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vrx_gz_directory, "/launch/competition.launch.py"]),
        launch_arguments={
            "world": world,
            "urdf": model_path,
            "extra_gz_args": "-v 0",  # verbose levels from 0 to 4
            # "spawn_pose": "-532.0,162.0,0.0,0.0,0.0,1.0",  # Original spawn point for sydney_regatta
            "spawn_pose": "-520.0,180.0,0.0,0.0,0.0,1.57",  # Original spawn point for sydney_regatta
        }.items(),
    )

    # LOCALIZATION
    sim_localization_params = os.path.join(asne_directory, "config", "sim_localization_params.yaml")

    lat, lon = (-33.7226, 150.6741)

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[
            ("gps/fix", "/wamv/sensors/gps/gps/fix"),
            ("imu/data", "/wamv/sensors/imu/imu/data"),
            ("odometry/filtered", "/odometry/filtered"),
        ],
        parameters=[
            sim_localization_params,
            {"datum": [lat, lon, 0.0]},  # comment out to set origin at first gps reading
        ],
        arguments=["--ros-args", "--log-level", "navsat_transform_node:=WARN"],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[sim_localization_params],
    )

    # Assume no sensor drift
    static_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    # VISUALIZATION/TESTING
    rviz_config_file = os.path.join(asne_directory, "rviz", "gazebo.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[{"use_sim_time": True}],
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    sim_bridge = Node(
        package="asne_pkg",
        executable="sim_bridge_node.py",
        name="sim_bridge_node",
        output="screen",
    )

    return [
        vrx_sim_launch,
        ekf_node,
        navsat_transform_node,
        static_transform_publisher_node,
        sim_bridge,
        rviz,
    ]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
    )

    rviz_arg = DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"])

    return LaunchDescription(
        [
            use_sim_time_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
