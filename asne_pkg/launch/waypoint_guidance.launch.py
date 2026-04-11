#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    waypoint_manager_node = Node(
        package="asne_pkg",
        executable="waypoint_manager_node.py",
        name="waypoint_manager_node",
        output="screen",
        parameters=[{
           "laps": 2,
           "waypoint_thresh": 10.0,
           "WP_A_lat": 36.8340,
           "WP_A_lon": -76.3700,
           "WP_B_lat": 36.8340,
           "WP_B_lon": -76.3700,
           "lane_width": 15.0,
           "turn_rad": 10.0,
           "turn_points": 6,
        }]
    )

    los_guidance_node = Node(
        package="asne_pkg",
        executable="los_guidance_node.py",
        name="los_guidance_node",
        output="screen",
    )

    return LaunchDescription([waypoint_manager_node, los_guidance_node])
