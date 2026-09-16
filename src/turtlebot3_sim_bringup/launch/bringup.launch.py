#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch TurtleBot3 in Gazebo with RViz visualization."""
    
    # Get package directory
    pkg_name = 'turtlebot3_sim_bringup'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # Include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    
    # Include RViz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'))
    
    ld.add_action(DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of the robot'))
    
    ld.add_action(DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position of the robot'))
    
    # Add launch files
    ld.add_action(gazebo_launch)
    ld.add_action(rviz_launch)
    
    return ld
