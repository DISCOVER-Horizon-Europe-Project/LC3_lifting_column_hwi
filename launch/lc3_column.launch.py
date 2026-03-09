#!/usr/bin/env python3
"""
Launch file for LC3 lifting column hardware interface.

This launch file:
1. Loads the URDF robot description
2. Starts the ros2_control controller manager
3. Spawns the joint state broadcaster
4. Spawns the column position controller
5. Launches RViz for visualization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Setup launch based on context parameters."""
    
    # Get package share directory
    pkg_share = FindPackageShare('lc3_hw_interface').perform(context)
    
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
            'simulation:=false',  # Always false - only use launch simulation param for controller selection
        ]
    )
    
    robot_description = {'robot_description': robot_description_content}

    # Controller configuration - select based on simulation parameter
    simulation = context.launch_configurations.get('simulation', 'false')
    
    if simulation == 'true':
        robot_controllers = os.path.join(pkg_share, 'config', 'lc3_controllers_jtc.yaml')
    else:
        robot_controllers = os.path.join(pkg_share, 'config', 'lc3_controllers_forward.yaml')

    rviz_config = os.path.join(pkg_share, 'config', 'lc3_column.rviz')

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='both',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
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
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        delay_column_controller_spawner,
    ]

    return nodes


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
        description='Run in simulation mode (uses JointTrajectoryController); default uses ForwardCommandController for real hardware'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with the package configuration file'
    )

    ld = LaunchDescription([
        use_mock_hardware_arg,
        simulation_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
    
    return ld
