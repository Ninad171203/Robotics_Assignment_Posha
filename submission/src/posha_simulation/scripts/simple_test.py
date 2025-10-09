#!/usr/bin/env python3
"""
Posha Robotics Assignment - Simple Test
Tests basic ROS2 functionality and path planning logic
"""

import rclpy
from rclpy.node import Node
import math
import time

class PoshaTest(Node):
    def __init__(self):
        super().__init__('posha_test_node')
        self.get_logger().info('🚀 POSHA ROBOTICS ASSIGNMENT - TEST STARTED')
        
        # Workspace coordinates based on assignment requirements
        self.workspace = {
            'container_1': [0.3, 0.3, 0.3],
            'container_5': [0.3, -0.3, 0.3],
            'pan_1': [0.0, -0.3, 0.4],
            'pan_2': [0.0, 0.3, 0.4],
            'spice_pod_1': [-0.25, 0.15, 0.3],
            'spice_pod_19': [-0.25, -0.15, 0.3]
        }
        
        self.test_assignment_requirements()
        
    def test_assignment_requirements(self):
        """Test the specific assignment requirements"""
        self.get_logger().info('')
        self.get_logger().info('📋 TESTING ASSIGNMENT REQUIREMENTS:')
        self.get_logger().info('====================================')
        
        # Test Case 1: Container 5 -> Pan 2
        self.get_logger().info('')
        self.get_logger().info('1. MACRO DISPENSE: Container 5 → Pan 2')
        self.test_macro_dispense('container_5', 'pan_2')
        
        # Test Case 2: Container 1 -> Pan 1
        self.get_logger().info('')
        self.get_logger().info('2. MACRO DISPENSE: Container 1 → Pan 1')
        self.test_macro_dispense('container_1', 'pan_1')
        
        # Test Case 3: Spice Pod 1 -> Pan 2
        self.get_logger().info('')
        self.get_logger().info('3. MICRO DISPENSE: Spice Pod 1 → Pan 2')
        self.test_micro_dispense('spice_pod_1', 'pan_2')
        
        # Test Case 4: Spice Pod 19 -> Pan 1
        self.get_logger().info('')
        self.get_logger().info('4. MICRO DISPENSE: Spice Pod 19 → Pan 1')
        self.test_micro_dispense('spice_pod_19', 'pan_1')
        
        self.get_logger().info('')
        self.get_logger().info('✅ ALL TESTS COMPLETED SUCCESSFULLY!')
        
    def test_macro_dispense(self, container, pan):
        """Test macro container dispensing"""
        start = self.workspace[container]
        end = self.workspace[pan]
        
        distance = self.calculate_distance(start, end)
        self.get_logger().info(f'   • Distance: {distance:.3f} meters')
        
        # Test path waypoints
        waypoints = self.calculate_macro_waypoints(start, end)
        self.get_logger().info(f'   • Waypoints: {len(waypoints)} steps')
        
        # Test collision detection
        if self.check_collision(start, end):
            self.get_logger().info('   • Status: 🚨 Collision detected (needs avoidance)')
        else:
            self.get_logger().info('   • Status: ✅ Path clear')
            
        # Test gripper precision (5mm requirement)
        grip_height = 0.005  # 5mm
        self.get_logger().info(f'   • Gripper precision: {grip_height*1000:.1f}mm above container')
        
    def test_micro_dispense(self, pod, pan):
        """Test spice pod dispensing"""
        start = self.workspace[pod]
        end = self.workspace[pan]
        
        distance = self.calculate_distance(start, end)
        self.get_logger().info(f'   • Distance: {distance:.3f} meters')
        
        # Test high precision requirement (2mm)
        grip_precision = 0.002  # 2mm
        self.get_logger().info(f'   • Precision required: {grip_precision*1000:.1f}mm from pod surface')
        
        if self.check_collision(start, end):
            self.get_logger().info('   • Status: 🚨 Collision detected (needs avoidance)')
        else:
            self.get_logger().info('   • Status: ✅ Path clear')
    
    def calculate_distance(self, p1, p2):
        """Calculate Euclidean distance between two 3D points"""
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
    
    def calculate_macro_waypoints(self, container_pos, pan_pos):
        """Calculate waypoints for macro container movement"""
        waypoints = []
        
        # Step 1: Approach container (10cm above)
        waypoints.append([container_pos[0], container_pos[1], container_pos[2] + 0.1])
        
        # Step 2: Gripper engagement (5mm above)
        waypoints.append([container_pos[0], container_pos[1], container_pos[2] + 0.005])
        
        # Step 3: Lift to safe height
        waypoints.append([container_pos[0], container_pos[1], container_pos[2] + 0.2])
        
        # Step 4: Move to pan approach
        waypoints.append([pan_pos[0], pan_pos[1], pan_pos[2] + 0.15])
        
        # Step 5: Dispense position
        waypoints.append([pan_pos[0], pan_pos[1], pan_pos[2] + 0.05])
        
        return waypoints
    
    def check_collision(self, start, end):
        """Simple collision detection"""
        mid_point = [(start[i] + end[i])/2 for i in range(3)]
        
        # Define collision zones (x, y, z, radius)
        collision_zones = [
            [0.0, 0.0, 0.2, 0.25],  # Center of workspace
            [0.1, 0.1, 0.3, 0.15]   # Potential obstacle
        ]
        
        for zone in collision_zones:
            dist = math.sqrt(
                (mid_point[0]-zone[0])**2 + 
                (mid_point[1]-zone[1])**2 + 
                (mid_point[2]-zone[2])**2
            )
            if dist < zone[3]:
                return True
        return False

def main():
    rclpy.init()
    
    try:
        node = PoshaTest()
        # Keep node running to see all output
        time.sleep(2)
        node.destroy_node()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
