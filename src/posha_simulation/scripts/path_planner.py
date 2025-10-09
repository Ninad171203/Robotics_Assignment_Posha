#!/usr/bin/env python3
"""
Main Path Planner for Posha Robotics Assignment
Handles both macro and micro dispensing operations
"""

import rclpy
from rclpy.node import Node
import math
import sys

class PoshaPathPlanner(Node):
    def __init__(self):
        super().__init__('posha_path_planner')
        
        self.get_logger().info('=== Initializing Posha Path Planner ===')
        
        # Workspace configuration
        self.workspace_config = {
            'macro_containers': {
                1: {'position': [0.3, 0.3, 0.3], 'color': 'blue'},
                2: {'position': [0.3, 0.1, 0.3], 'color': 'red'}, 
                3: {'position': [0.3, -0.1, 0.3], 'color': 'green'},
                4: {'position': [0.3, -0.3, 0.3], 'color': 'yellow'},
                5: {'position': [0.5, 0.0, 0.3], 'color': 'purple'}
            },
            'pans': {
                1: {'position': [0.0, -0.3, 0.4], 'radius': 0.13},
                2: {'position': [0.0, 0.3, 0.4], 'radius': 0.13}
            },
            'spice_pods': self.generate_spice_pod_positions()
        }
        
        self.print_workspace_info()
        
    def generate_spice_pod_positions(self):
        """Generate circular arrangement for 20 spice pods"""
        pods = {}
        radius = 0.2
        center_x, center_y = -0.4, 0.0
        base_z = 0.3
        
        for i in range(1, 21):
            angle = 2 * math.pi * (i-1) / 20
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            pods[i] = {'position': [x, y, base_z], 'angle': angle}
            
        return pods
    
    def print_workspace_info(self):
        """Print detailed workspace information"""
        self.get_logger().info('=== WORKSPACE CONFIGURATION ===')
        
        self.get_logger().info('MACRO CONTAINERS:')
        for id, config in self.workspace_config['macro_containers'].items():
            pos = config['position']
            self.get_logger().info(f'  Container {id}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]')
            
        self.get_logger().info('PANS:')
        for id, config in self.workspace_config['pans'].items():
            pos = config['position']
            self.get_logger().info(f'  Pan {id}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]')
            
        self.get_logger().info('SPICE PODS (first 5):')
        for i in range(1, 6):
            if i in self.workspace_config['spice_pods']:
                pos = self.workspace_config['spice_pods'][i]['position']
                self.get_logger().info(f'  Pod {i}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]')
    
    def plan_macro_dispense(self, container_id, pan_id):
        """Plan path for macro container dispensing"""
        self.get_logger().info(f'=== PLANNING MACRO DISPENSE: Container {container_id} -> Pan {pan_id} ===')
        
        if container_id not in self.workspace_config['macro_containers']:
            self.get_logger().error(f'Container {container_id} not found!')
            return False
            
        if pan_id not in self.workspace_config['pans']:
            self.get_logger().error(f'Pan {pan_id} not found!')
            return False
        
        container_pos = self.workspace_config['macro_containers'][container_id]['position']
        pan_pos = self.workspace_config['pans'][pan_id]['position']
        
        # Calculate path waypoints
        waypoints = self.calculate_macro_waypoints(container_pos, pan_pos)
        
        self.get_logger().info(f'Planned {len(waypoints)} waypoints')
        for i, wp in enumerate(waypoints):
            self.get_logger().info(f'  Waypoint {i}: [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}]')
        
        return True
    
    def calculate_macro_waypoints(self, container_pos, pan_pos):
        """Calculate waypoints for macro container movement"""
        waypoints = []
        
        # Step 1: Approach container (10cm above)
        approach = [container_pos[0], container_pos[1], container_pos[2] + 0.1]
        waypoints.append(approach)
        
        # Step 2: Gripper engagement (5mm above)
        grip = [container_pos[0], container_pos[1], container_pos[2] + 0.005]
        waypoints.append(grip)
        
        # Step 3: Lift to safe height
        lift = [container_pos[0], container_pos[1], container_pos[2] + 0.2]
        waypoints.append(lift)
        
        # Step 4: Move to pan approach
        pan_approach = [pan_pos[0], pan_pos[1], pan_pos[2] + 0.15]
        waypoints.append(pan_approach)
        
        # Step 5: Dispense position
        dispense = [pan_pos[0], pan_pos[1], pan_pos[2] + 0.05]
        waypoints.append(dispense)
        
        return waypoints

def main():
    rclpy.init()
    
    planner = PoshaPathPlanner()
    
    # Test the planned operations
    try:
        # Test Case 1: Container 5 -> Pan 2
        planner.plan_macro_dispense(5, 2)
        
        # Test Case 2: Container 1 -> Pan 1  
        planner.plan_macro_dispense(1, 1)
        
        # Keep node alive
        rclpy.spin(planner)
        
    except KeyboardInterrupt:
        planner.get_logger().info('=== Path planner shutdown ===')
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
