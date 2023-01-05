#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    aruco_dir = get_package_share_directory('ros2_aruco')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ld = LaunchDescription()

    declare_docking = Node(
        package='marker_docking',
        executable='marker_docking',
        name='marker_docking',
        output='screen'
    )

    declare_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aruco_dir, 'launch', 'aruco.launch.py'))
    )

    ld.add_action(declare_aruco)
    ld.add_action(declare_docking)

    return ld
