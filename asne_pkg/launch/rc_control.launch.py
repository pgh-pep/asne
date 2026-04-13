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
    # asne_directory = get_package_share_directory("asne_pkg")

    serial_comm_node = Node(
        package="asne_pkg",
        executable="serial_comm_node.py",
        name="serial_comm_node",
        output="screen",
        parameters=[
            {"serial_port": "/dev/ttyUSB0"},
            {"baud": 115200},
        ],
    )

    rc_decoder_node = Node(
        package="asne_pkg",
        executable="rc_decoder_node.py",
        name="rc_decoder_node",
        output="screen",
    )

    manual_control_node = Node(
        package="asne_pkg",
        executable="manual_control_node.py",
        name="manual_control_node",
        output="screen",
        # parameters=[
        #     {"max_linear_velocity": 1.0},
        # ],
    )

    motion_output_mux = Node(
        package="asne_pkg",
        executable="motion_output_mux.py",
        name="motion_output_mux",
    )

    return [serial_comm_node, rc_decoder_node, manual_control_node, motion_output_mux]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
