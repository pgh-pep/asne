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
           "WP_A_lat": -33.722600,
           "WP_A_lon": 150.674123,
           "WP_B_lat": -33.72142674509712,
           "WP_B_lon": 150.67567838126456,
           "lane_width": 15.0,
           "turn_rad": 20.0,
           "turn_points": 6,
        }]
    )

    velocity_heading_mux = Node(
        package="asne_pkg",
        executable="velocity_heading_mux.py",
        name="velocity_heading_mux",
    )

    pd_heading_node = Node(
        package="asne_pkg",
        executable="pd_heading_node.py",
        name="pd_heading_node",
        output="screen"
    )

    los_guidance_node = Node(
        package="asne_pkg",
        executable="los_guidance_node.py",
        name="los_guidance_node",
        output="screen",
    )

    return LaunchDescription([waypoint_manager_node, los_guidance_node, velocity_heading_mux, pd_heading_node])
