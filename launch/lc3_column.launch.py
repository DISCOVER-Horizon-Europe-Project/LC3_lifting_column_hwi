#!/usr/bin/env python3
"""
Launch file for LC3 lifting column hardware interface.

This launch file:
1. Loads the URDF robot description
2. Starts the ros2_control controller manager
3. Spawns the joint state broadcaster
4. Spawns the column position controller
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware (GenericSystem) instead of real LC3 hardware'
    )
    
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in Gazebo simulation mode'
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('lc3_hw_interface'), 'urdf', 'lc3_column.urdf.xacro']
            ),
            ' ',
            'use_mock_hardware:=',
            LaunchConfiguration('use_mock_hardware'),
            ' ',
            'simulation:=',
            LaunchConfiguration('simulation'),
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('lc3_hw_interface'), 'config', 'lc3_controllers.yaml']
    )

    # Controller manager node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Column position controller spawner (delayed to ensure controller manager is ready)
    column_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['column_position_controller', '--controller-manager', '/controller_manager'],
    )

    # Delay column controller spawner after joint state broadcaster
    delay_column_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[column_controller_spawner],
        )
    )

    nodes = [
        use_mock_hardware_arg,
        simulation_arg,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_column_controller_spawner,
    ]

    return LaunchDescription(nodes)
