#!/usr/bin/env python3
"""
Launch file for Posha Simulation
Starts Gazebo, RViz, and loads the workspace
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    launch_description = LaunchDescription()
    
    # Start Robot State Publisher with simple workspace
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[['~/posha_ros2_ws/src/posha_simulation/urdf/simple_workspace.urdf']]
    )
    
    # Start Joint State Publisher (for movable joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # Start RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '~/posha_ros2_ws/src/posha_simulation/config/rviz/simulation.rviz']
    )
    
    # Start our test node
    test_node = Node(
        package='posha_simulation',
        executable='simple_test.py',
        name='posha_test_node',
        output='screen'
    )
    
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(joint_state_publisher)
    launch_description.add_action(rviz2)
    launch_description.add_action(test_node)
    
    return launch_description
