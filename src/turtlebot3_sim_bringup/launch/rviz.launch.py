#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch RViz2 for TurtleBot3 visualization."""
    
    # Get package directory
    pkg_name = 'turtlebot3_sim_bringup'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # RViz config file
    rviz_config_file = os.path.join(
        pkg_dir,
        'config',
        'turtlebot3.rviz'
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'))
    
    # Add RViz node
    ld.add_action(rviz_node)
    
    return ld
