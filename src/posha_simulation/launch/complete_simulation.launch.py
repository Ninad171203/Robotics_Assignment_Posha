#!/usr/bin/env python3
"""
Complete simulation launch file for Posha Robotics Assignment
Launches Gazebo, RViz, and all necessary nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable

def generate_launch_description():
    
    launch_description = LaunchDescription()
    
    # Start Robot State Publisher with workspace URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[FindExecutable.find('ros2'), 'run', 'posha_simulation', 'urdf/simple_workspace.urdf']
    )
    
    # Start Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # Start RViz2 with configuration
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', FindExecutable.find('ros2'), 'run', 'posha_simulation', 'config/rviz/simulation.rviz']
    )
    
    # Start Path Planner Node
    path_planner = Node(
        package='posha_simulation',
        executable='advanced_planner.py',
        name='advanced_path_planner',
        output='screen'
    )
    
    # Start Simple Test Node
    test_node = Node(
        package='posha_simulation',
        executable='simple_test.py',
        name='assignment_test_node',
        output='screen'
    )
    
    # Add all nodes to launch description
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    launch_description.add_action(rviz2)
    launch_description.add_action(path_planner)
    launch_description.add_action(test_node)
    
    return launch_description
