#!/usr/bin/env python3
"""
Complete simulation launch file for Posha Robotics Assignment
Launches Gazebo, RViz, and all necessary nodes - FIXED VERSION
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    # Get package share directory
    pkg_path = FindPackageShare('posha_simulation').find('posha_simulation')
    
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
            'world': PathJoinSubstitution([
                pkg_path, 'worlds', 'empty.world'
            ])
        }.items()
    )

    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'posha_robot',
            '-file', PathJoinSubstitution([pkg_path, 'urdf', 'simple_workspace.urdf']),
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # Start Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(PathJoinSubstitution([pkg_path, 'urdf', 'simple_workspace.urdf']), 'r').read(),
            'use_sim_time': True
        }]
    )

    # Start Joint State Publisher GUI
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Start RViz2 with configuration
    rviz_config_path = PathJoinSubstitution([pkg_path, 'config', 'simulation.rviz'])
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
        executable='advanced_planner.py',
        name='advanced_path_planner',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Start Test Node
    test_node = Node(
        package='posha_simulation',
        executable='simple_test.py',
        name='assignment_test_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Add all nodes to launch description
    launch_description.add_action(gazebo_launch)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    launch_description.add_action(spawn_entity)
    launch_description.add_action(rviz2)
    launch_description.add_action(path_planner)
    launch_description.add_action(test_node)

    return launch_description