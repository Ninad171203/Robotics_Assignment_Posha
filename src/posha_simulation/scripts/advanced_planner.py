#!/usr/bin/env python3
"""
Advanced Path Planner for Piper Robot - Posha Assignment
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Point, Pose
from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import JointState

class PiperPlanner(Node):
    def __init__(self):
        super().__init__('piper_path_planner')
        
        self.get_logger().info("🚀 Piper Path Planner Initialized")
        
        # Workspace coordinates (same as assignment)
        self.workspace = {
            'container_1': [0.3, 0.3, 0.3],
            'container_5': [0.3, -0.3, 0.3],
            'pan_1': [0.0, -0.3, 0.4],
            'pan_2': [0.0, 0.3, 0.4],
            'spice_pod_1': [-0.25, 0.15, 0.3],
            'spice_pod_19': [-0.25, -0.15, 0.3]
        }
        
        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = [0.0] * 8
        
        self.get_logger().info("✅ Piper Planner Ready for Posha Assignment Tasks")

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        if len(msg.position) >= 8:
            self.current_joint_positions = list(msg.position)

    def calculate_distance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

    def calculate_macro_waypoints(self, start, end):
        """Calculate waypoints for macro container movement with Piper"""
        waypoints = []
        
        # Step 1: Approach container (15cm above)
        waypoints.append([start[0], start[1], start[2] + 0.15])
        
        # Step 2: Gripper engagement (2cm above)
        waypoints.append([start[0], start[1], start[2] + 0.02])
        
        # Step 3: Lift to safe height
        waypoints.append([start[0], start[1], start[2] + 0.25])
        
        # Step 4: Move to pan approach
        waypoints.append([end[0], end[1], end[2] + 0.2])
        
        # Step 5: Dispense position
        waypoints.append([end[0], end[1], end[2] + 0.03])
        
        return waypoints

    def calculate_micro_waypoints(self, start, end):
        """Calculate waypoints for micro spice pod movement"""
        waypoints = []
        
        # Step 1: Approach pod (10cm above)
        waypoints.append([start[0], start[1], start[2] + 0.1])
        
        # Step 2: Precision approach (1cm above)
        waypoints.append([start[0], start[1], start[2] + 0.01])
        
        # Step 3: Lift to safe height
        waypoints.append([start[0], start[1], start[2] + 0.15])
        
        # Step 4: Move to pan approach
        waypoints.append([end[0], end[1], end[2] + 0.12])
        
        # Step 5: Precision dispense (2mm above)
        waypoints.append([end[0], end[1], end[2] + 0.002])
        
        return waypoints

    def inverse_kinematics(self, target_pose):
        """
        Simplified inverse kinematics for Piper robot
        This is a placeholder - you'll need to implement proper IK
        """
        x, y, z = target_pose
        
        # Simple heuristic for joint angles
        # In real implementation, use proper IK solver
        joint_angles = [
            math.atan2(y, x),  # joint1 - base rotation
            math.pi/4,         # joint2 - shoulder
            -math.pi/4,        # joint3 - elbow  
            0.0,               # joint4 - wrist1
            math.pi/2,         # joint5 - wrist2
            0.0,               # joint6 - wrist3
            0.0,               # joint7 - gripper left
            0.0                # joint8 - gripper right
        ]
        
        return joint_angles

    def plan_macro_dispense(self, container_name, pan_name):
        """Plan macro dispense task"""
        self.get_logger().info(f"📦 Planning MACRO dispense: {container_name} → {pan_name}")
        
        start = self.workspace[container_name]
        end = self.workspace[pan_name]
        
        waypoints = self.calculate_macro_waypoints(start, end)
        distance = self.calculate_distance(start, end)
        
        self.get_logger().info(f"   • Distance: {distance:.3f}m")
        self.get_logger().info(f"   • Waypoints: {len(waypoints)}")
        self.get_logger().info(f"   • Precision: 2.0cm gripper engagement")
        
        # Convert waypoints to joint trajectories
        joint_trajectory = []
        for wp in waypoints:
            joint_angles = self.inverse_kinematics(wp)
            joint_trajectory.append(joint_angles)
            
        return joint_trajectory

    def plan_micro_dispense(self, pod_name, pan_name):
        """Plan micro dispense task"""
        self.get_logger().info(f"🧂 Planning MICRO dispense: {pod_name} → {pan_name}")
        
        start = self.workspace[pod_name]
        end = self.workspace[pan_name]
        
        waypoints = self.calculate_micro_waypoints(start, end)
        distance = self.calculate_distance(start, end)
        
        self.get_logger().info(f"   • Distance: {distance:.3f}m")
        self.get_logger().info(f"   • Waypoints: {len(waypoints)}")
        self.get_logger().info(f"   • Precision: 2.0mm from pod surface")
        
        # Convert waypoints to joint trajectories
        joint_trajectory = []
        for wp in waypoints:
            joint_angles = self.inverse_kinematics(wp)
            joint_trajectory.append(joint_angles)
            
        return joint_trajectory

def main():
    rclpy.init()
    planner = PiperPlanner()
    
    try:
        # Test planning for all assignment scenarios
        planner.get_logger().info("🎯 Starting Posha Assignment Planning...")
        
        # Macro dispense scenarios
        planner.plan_macro_dispense('container_5', 'pan_2')
        planner.plan_macro_dispense('container_1', 'pan_1')
        
        # Micro dispense scenarios  
        planner.plan_micro_dispense('spice_pod_1', 'pan_2')
        planner.plan_micro_dispense('spice_pod_19', 'pan_1')
        
        planner.get_logger().info("✅ All planning tasks completed!")
        
        rclpy.spin(planner)
        
    except KeyboardInterrupt:
        planner.get_logger().info("🛑 Planner shutdown requested")
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()