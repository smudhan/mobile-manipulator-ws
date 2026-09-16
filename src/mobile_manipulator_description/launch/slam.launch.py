from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    description_pkg = get_package_share_directory(
        'mobile_manipulator_description'
    )

    gazebo_launch = os.path.join(
        description_pkg,
        'launch',
        'mobile_manipulator_gazebo.launch.py'
    )

    slam_params = os.path.join(
        description_pkg,
        'config',
        'slam_toolbox.yaml'
    )

    return LaunchDescription([

        # Start hospital world + combined mobile manipulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),
    ])