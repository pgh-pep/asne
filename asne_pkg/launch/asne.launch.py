#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():   
    asne_directory = get_package_share_directory("asne_pkg")
    
    launch_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(asne_directory, "launch", "localization.launch.py"),
           
        ])
    )

    launch_motion_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(asne_directory, "launch", "motion_control.launch.py"),           
        ])
    )

    launch_rc_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(asne_directory, "launch", "rc_control.launch.py"),           
        ])
    )

    launch_system_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(asne_directory, "launch", "system_control.launch.py"),           
        ])
    )

    launch_motor_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(asne_directory, "launch", "motor_control.launch.py"),           
        ])
    )

    return LaunchDescription(
        [
            launch_localization,
            launch_motion_control,
            launch_rc_control,
            launch_system_control,
            launch_motor_control
        ]
    )
