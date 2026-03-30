#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# run these beforehand:
# sudo ip link set can0 type can bitrate 250000
# sudo ip link set up can0


def launch_setup(context, *args, **kwargs):
    asne_directory = get_package_share_directory("asne_pkg")
    dcf_file = os.path.join(asne_directory, "config", "sevcon_working.dcf")

    motor_control_node = Node(
        package="asne_pkg",
        executable="motor_control_node.py",
        name="motor_control_node",
        output="screen",
        parameters=[
            {"dcf_path": dcf_file},
            {"baud": 115200},
        ],
    )

    return [motor_control_node]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
