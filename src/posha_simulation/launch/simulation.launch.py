#!/usr/bin/env python3
"""
Complete simulation launch file for Posha Robotics Assignment
With Piper robot integration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # Get package paths
    pkg_path = FindPackageShare('posha_simulation').find('posha_simulation')
    piper_pkg_path = FindPackageShare('piper_description').find('piper_description')
    
    launch_description = LaunchDescription()

    # Launch Gazebo Empty World
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([pkg_path, 'worlds', 'empty.world']),
            'verbose': 'true'
        }.items()
    )

    # Robot State Publisher for Piper robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(PathJoinSubstitution([piper_pkg_path, 'urdf', 'piper.urdf']), 'r').read(),
            'use_sim_time': True
        }]
    )

    # Spawn Piper Robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'piper_robot',
            '-file', PathJoinSubstitution([piper_pkg_path, 'urdf', 'piper.urdf']),
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Spawn Workspace Objects
    spawn_workspace = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'workspace_objects',
            '-file', PathJoinSubstitution([pkg_path, 'urdf', 'workspace.urdf']),
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # Start Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Start RViz2 with configuration
    rviz_config_path = PathJoinSubstitution([pkg_path, 'config', 'piper_simulation.rviz'])
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    # Start Path Planner Node
    path_planner = Node(
        package='posha_simulation',
        executable='piper_planner.py',
        name='piper_path_planner',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Start Controller Node
    controller_node = Node(
        package='posha_simulation',
        executable='piper_controller.py',
        name='piper_controller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Add all nodes to launch description
    launch_description.add_action(gazebo_launch)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    launch_description.add_action(spawn_robot)
    launch_description.add_action(spawn_workspace)
    launch_description.add_action(rviz2)
    launch_description.add_action(path_planner)
    launch_description.add_action(controller_node)

    return launch_description