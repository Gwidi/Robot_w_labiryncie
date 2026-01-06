import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_explorer',
            executable='explorer',
            name='explorer_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        parameters=[{
            'use_sim_time': True,
            'max_laser_range': 3.5,
            'min_laser_range': 0.12
        }]
)
    ])