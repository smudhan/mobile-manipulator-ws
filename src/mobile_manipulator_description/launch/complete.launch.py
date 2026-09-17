#!/usr/bin/env python3

import os
import re
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---------------------------------------------------------
    # Package paths
    # ---------------------------------------------------------

    hospital_pkg = get_package_share_directory(
        'aws_robomaker_hospital_world'
    )

    gazebo_ros_pkg = get_package_share_directory(
        'gazebo_ros'
    )

    description_pkg = get_package_share_directory(
        'mobile_manipulator_description'
    )

    # ---------------------------------------------------------
    # Paths
    # ---------------------------------------------------------

    world = os.path.join(
        hospital_pkg,
        'worlds',
        'hospital.world'
    )

    model_path = os.path.join(
        hospital_pkg,
        'fuel_models'
    )

    hospital_models_path = os.path.join(
        hospital_pkg,
        'models'
    )

    xacro_file = os.path.join(
        description_pkg,
        'urdf',
        'mobile_manipulator.urdf.xacro'
    )

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    x_pose = LaunchConfiguration(
        'x_pose'
    )

    y_pose = LaunchConfiguration(
        'y_pose'
    )

    # ---------------------------------------------------------
    # Gazebo model path
    # ---------------------------------------------------------

    gazebo_model_path = os.pathsep.join([
        model_path,
        hospital_models_path,
        os.environ.get('GAZEBO_MODEL_PATH', '')
    ])

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # ---------------------------------------------------------
    # Gazebo server
    # ---------------------------------------------------------

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_pkg,
                'launch',
                'gzserver.launch.py'
            )
        ),
        launch_arguments={
            'world': world
        }.items()
    )

    # ---------------------------------------------------------
    # Gazebo GUI
    # ---------------------------------------------------------

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_pkg,
                'launch',
                'gzclient.launch.py'
            )
        )
    )

    # ---------------------------------------------------------
    # Robot description
    # ---------------------------------------------------------

    robot_description_xml = subprocess.check_output(
        ['xacro', xacro_file],
        text=True
    )

    # gazebo_ros2_control on this system has trouble parsing
    # XML comments when robot_description is passed internally.
    robot_description_xml = re.sub(
        r'<!--.*?-->',
        '',
        robot_description_xml,
        flags=re.DOTALL
    )

    robot_description = ParameterValue(
        robot_description_xml,
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # ---------------------------------------------------------
    # Spawn combined robot
    # ---------------------------------------------------------

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'mobile_manipulator',
            '-topic',
            'robot_description',
            '-x',
            '1.0',
            '-y',
            '1.0',
            '-z',
            '0.08'
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Joint State Broadcaster
    # ---------------------------------------------------------

    load_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner',
                    'joint_state_broadcaster'
                ],
                output='screen'
            )
        ]
    )

    # ---------------------------------------------------------
    # Joint Trajectory Controller
    # ---------------------------------------------------------

    load_joint_trajectory_controller = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner',
                    'joint_trajectory_controller'
                ],
                output='screen'
            )
        ]
    )

    # ---------------------------------------------------------
    # Nav2 parameters
    # ---------------------------------------------------------

    nav2_params = os.path.join(
        description_pkg,
        'config',
        'nav2.yaml'
    )

    # ---------------------------------------------------------
    # Nav2 Map Server
    # ---------------------------------------------------------

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params]
    )

    # ---------------------------------------------------------
    # Nav2 AMCL
    # ---------------------------------------------------------

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )

    # ---------------------------------------------------------
    # Nav2 Planner
    # ---------------------------------------------------------

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    # ---------------------------------------------------------
    # Nav2 Controller
    # ---------------------------------------------------------

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    # ---------------------------------------------------------
    # Nav2 BT Navigator
    # ---------------------------------------------------------

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    # ---------------------------------------------------------
    # Nav2 Behavior Server
    # ---------------------------------------------------------

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )

    # ---------------------------------------------------------
    # Nav2 Lifecycle Manager
    # ---------------------------------------------------------

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            nav2_params,
            {
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'behavior_server'
                ]
            }
        ]
    )

    # ---------------------------------------------------------
    # RViz2
    # ---------------------------------------------------------

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # ---------------------------------------------------------
    # Launch description
    # ---------------------------------------------------------

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0'
        ),

        # Gazebo
        set_gazebo_model_path,
        gazebo_server,
        gazebo_client,

        # Robot
        robot_state_publisher,

        TimerAction(
            period=5.0,
            actions=[spawn_robot]
        ),

        # Controllers
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,

        # Nav2
        map_server,
        amcl,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager,

        # RViz
        rviz2,
    ])