#!/usr/bin/env python3

from launch_ros.actions import Node
from launch import LaunchDescription
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

    # LOCALIZATION
    localization_params = os.path.join(asne_directory, "config", "localization_params.yaml")

    lat, lon = (40.444488, -79.957622) # TODO: CHANGE

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
        parameters=[
            localization_params,
            {"datum": [lat, lon, 0.0]},  # comment out to set origin at first gps reading
        ],
        arguments=["--ros-args", "--log-level", "navsat_transform_node:=WARN"],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[localization_params],
    )

    static_odom_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "map", "--child-frame-id","odom"],
    )

    static_base_link_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base_link_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "odom","--child-frame-id","base_link"],
    )

    static_imu_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_link_to_imu_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "base_link","--child-frame-id","imu_link"],

    )

    static_gps_transform_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_link_to_gps_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "base_link","--child-frame-id","gps"],

    )

    imu_driver_node = Node(
        package="asne_pkg",
        executable="imu_driver_node.py",
        name="imu_driver_node"
    )

    gps_serial_driver = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        name="gps_serial_driver",
        output="screen",
        parameters=[
            {"port": "/dev/ttyACM0"},  # /dev/ttyACM0 or /dev/ttyUSB1
            {"baud": 38400},
        ],
        remappings=[
            ("/fix", "/gps/fix"),
            ("/vel", "/gps/vel"),
            ("/time_reference", "/gps/time_reference"),
        ],
        # arguments=["--ros-args", "--log-level", "debug"],
    )

    return [
        ekf_node,
        gps_serial_driver,
        navsat_transform_node,
        imu_driver_node,
        static_odom_transform_publisher_node,
        static_base_link_transform_publisher_node,
        static_imu_transform_publisher_node,
        static_gps_transform_publisher_node,
    ]


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
