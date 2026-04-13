#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # asne_directory = get_package_share_directory("asne_pkg")

    system_controller_node = Node(
        package="asne_pkg",
        executable="system_controller.py",
        name="system_controller_node"
    )

    gps_failsafe_node = Node(
        package="asne_pkg",
        executable="gps_failsafe_node.py",
        name="gps_failsafe_node"
    )

    return [
        gps_failsafe_node,
        system_controller_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup)
        ]
    )
