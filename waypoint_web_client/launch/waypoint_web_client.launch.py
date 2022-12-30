import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    waypoint_web_client_dir = get_package_share_directory('waypoint_web_client')
    sparo_turtlebot4_navigation_dir = get_package_share_directory('sparo_turtlebot4_navigation')

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sparo_turtlebot4_navigation_dir, 'launch', 'map_server.launch.py')
            )
        )
    )
    
    ld.add_action(
        Node(
            package='waypoint_web_client',
            executable='waypoint_web_client',
            name='waypoint_web_client',
            output='screen'
        )
    )

    return ld