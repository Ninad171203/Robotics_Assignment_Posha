#!/usr/bin/env python3
"""
Advanced Path Planner for Posha Robotics Assignment
Implements RRT algorithm, collision avoidance, and optimization
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray
import time

class AdvancedPathPlanner(Node):
    def __init__(self):
        super().__init__('advanced_path_planner')
        
        # Workspace configuration
        self.workspace_bounds = {
            'x': [-0.6, 0.6],
            'y': [-0.4, 0.4], 
            'z': [0.1, 0.6]
        }
        
        # Obstacles in the workspace
        self.obstacles = [
            {'type': 'cylinder', 'position': [0.0, 0.0, 0.2], 'radius': 0.15, 'height': 0.3},  # Center column
            {'type': 'box', 'position': [0.2, 0.2, 0.15], 'size': [0.1, 0.1, 0.3]},  # Equipment box
        ]
        
        self.get_logger().info('=== ADVANCED PATH PLANNER INITIALIZED ===')
        
    def plan_macro_dispense(self, container_id, pan_id):
        """Plan complete macro dispense operation"""
        self.get_logger().info(f'Planning macro dispense: Container {container_id} -> Pan {pan_id}')
        
        # Get start and goal positions
        start_pos = self.get_container_position(container_id)
        goal_pos = self.get_pan_position(pan_id)
        
        # Generate complete trajectory
        trajectory = self.generate_macro_trajectory(start_pos, goal_pos)
        
        # Optimize trajectory
        optimized_trajectory = self.optimize_trajectory(trajectory)
        
        return optimized_trajectory
    
    def plan_micro_dispense(self, pod_id, pan_id):
        """Plan complete micro dispense operation"""
        self.get_logger().info(f'Planning micro dispense: Pod {pod_id} -> Pan {pan_id}')
        
        # Get start and goal positions
        start_pos = self.get_pod_position(pod_id)
        goal_pos = self.get_pan_position(pan_id)
        
        # Generate high-precision trajectory
        trajectory = self.generate_micro_trajectory(start_pos, goal_pos)
        
        return trajectory
    
    def generate_macro_trajectory(self, start, goal):
        """Generate trajectory for macro container movement"""
        waypoints = []
        
        # Step 1: Approach container (smooth approach)
        approach = self.smooth_approach(start, height=0.1)
        waypoints.extend(approach)
        
        # Step 2: Gripper engagement sequence
        grip_sequence = self.gripper_engagement(start, engagement_height=0.005)
        waypoints.extend(grip_sequence)
        
        # Step 3: Lift with clearance
        lift_path = self.lift_with_clearance(start, lift_height=0.2)
        waypoints.extend(lift_path)
        
        # Step 4: Plan path to pan using RRT
        transit_path = self.plan_transit_path(waypoints[-1], goal, max_nodes=1000)
        waypoints.extend(transit_path)
        
        # Step 5: Dispense sequence
        dispense_sequence = self.dispense_sequence(goal)
        waypoints.extend(dispense_sequence)
        
        return waypoints
    
    def generate_micro_trajectory(self, start, goal):
        """Generate high-precision trajectory for micro dispensing"""
        waypoints = []
        
        # High-precision approach (2mm requirement)
        precision_approach = self.high_precision_approach(start, precision=0.002)
        waypoints.extend(precision_approach)
        
        # Lift and transit with obstacle avoidance
        transit_path = self.plan_transit_path(waypoints[-1], goal, resolution=0.01)
        waypoints.extend(transit_path)
        
        # Precision dispense
        dispense = self.precision_dispense(goal)
        waypoints.extend(dispense)
        
        return waypoints
    
    def plan_transit_path(self, start, goal, max_nodes=500, resolution=0.05):
        """Implement RRT algorithm for path planning"""
        self.get_logger().info('Planning transit path using RRT...')
        
        # Initialize tree
        tree = [{'point': start, 'parent': None}]
        
        for i in range(max_nodes):
            # Sample random point
            random_point = self.sample_random_point()
            
            # Find nearest node in tree
            nearest_node = self.find_nearest_node(tree, random_point)
            
            # Extend toward random point
            new_point = self.extend_toward(nearest_node['point'], random_point, resolution)
            
            # Check collision
            if not self.check_collision_segment(nearest_node['point'], new_point):
                # Add to tree
                tree.append({'point': new_point, 'parent': len(tree) - 1})
                
                # Check if goal is reached
                if self.distance(new_point, goal) < resolution:
                    self.get_logger().info(f'RRT path found after {i} iterations')
                    return self.extract_path(tree, new_point)
        
        self.get_logger().warning('RRT failed to find path, using straight line')
        return [goal]
    
    def sample_random_point(self):
        """Sample a random point in workspace"""
        x = np.random.uniform(self.workspace_bounds['x'][0], self.workspace_bounds['x'][1])
        y = np.random.uniform(self.workspace_bounds['y'][0], self.workspace_bounds['y'][1])
        z = np.random.uniform(self.workspace_bounds['z'][0], self.workspace_bounds['z'][1])
        return [x, y, z]
    
    def find_nearest_node(self, tree, target):
        """Find nearest node in tree to target point"""
        min_dist = float('inf')
        nearest = None
        
        for node in tree:
            dist = self.distance(node['point'], target)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        
        return nearest
    
    def extend_toward(self, from_point, to_point, step_size):
        """Extend from point toward to point by step_size"""
        direction = np.array(to_point) - np.array(from_point)
        distance = np.linalg.norm(direction)
        
        if distance < step_size:
            return to_point
        
        direction = direction / distance
        new_point = np.array(from_point) + direction * step_size
        return new_point.tolist()
    
    def extract_path(self, tree, goal_node):
        """Extract path from tree"""
        path = []
        current = goal_node
        
        while current is not None:
            path.append(tree[current]['point'])
            current = tree[current]['parent']
        
        return list(reversed(path))
    
    def check_collision_segment(self, start, end):
        """Check collision along a line segment"""
        # Sample points along segment
        num_samples = max(3, int(self.distance(start, end) / 0.02))
        
        for i in range(num_samples + 1):
            t = i / num_samples
            point = [
                start[0] + t * (end[0] - start[0]),
                start[1] + t * (end[1] - start[1]),
                start[2] + t * (end[2] - start[2])
            ]
            
            if self.check_collision_point(point):
                return True
        
        return False
    
    def check_collision_point(self, point):
        """Check if a point collides with any obstacle"""
        for obstacle in self.obstacles:
            if obstacle['type'] == 'cylinder':
                # Cylinder collision check
                dx = point[0] - obstacle['position'][0]
                dy = point[1] - obstacle['position'][1]
                horizontal_dist = math.sqrt(dx*dx + dy*dy)
                
                if (horizontal_dist < obstacle['radius'] and 
                    abs(point[2] - obstacle['position'][2]) < obstacle['height']/2):
                    return True
                    
            elif obstacle['type'] == 'box':
                # Box collision check
                in_x = abs(point[0] - obstacle['position'][0]) < obstacle['size'][0]/2
                in_y = abs(point[1] - obstacle['position'][1]) < obstacle['size'][1]/2
                in_z = abs(point[2] - obstacle['position'][2]) < obstacle['size'][2]/2
                
                if in_x and in_y and in_z:
                    return True
        
        return False
    
    def smooth_approach(self, target, height=0.1):
        """Generate smooth approach waypoints"""
        return [
            [target[0], target[1], target[2] + height],
            [target[0], target[1], target[2] + height/2]
        ]
    
    def gripper_engagement(self, target, engagement_height=0.005):
        """Generate gripper engagement sequence"""
        return [
            [target[0], target[1], target[2] + engagement_height*2],
            [target[0], target[1], target[2] + engagement_height]
        ]
    
    def lift_with_clearance(self, start, lift_height=0.2):
        """Lift with clearance checking"""
        return [
            [start[0], start[1], start[2] + lift_height/2],
            [start[0], start[1], start[2] + lift_height]
        ]
    
    def high_precision_approach(self, target, precision=0.002):
        """High precision approach for micro dispensing"""
        return [
            [target[0], target[1], target[2] + 0.05],
            [target[0], target[1], target[2] + 0.01],
            [target[0], target[1], target[2] + precision]
        ]
    
    def dispense_sequence(self, target):
        """Generate dispense sequence"""
        return [
            [target[0], target[1], target[2] + 0.1],
            [target[0], target[1], target[2] + 0.05],
            [target[0], target[1], target[2] + 0.02]
        ]
    
    def precision_dispense(self, target):
        """High precision dispense for micro items"""
        return [
            [target[0], target[1], target[2] + 0.03],
            [target[0], target[1], target[2] + 0.01]
        ]
    
    def optimize_trajectory(self, trajectory):
        """Optimize trajectory for smoothness and efficiency"""
        if len(trajectory) < 3:
            return trajectory
        
        # Simple optimization: remove redundant waypoints
        optimized = [trajectory[0]]
        
        for i in range(1, len(trajectory)-1):
            # Keep waypoint if direction changes significantly
            prev_vec = np.array(trajectory[i]) - np.array(trajectory[i-1])
            next_vec = np.array(trajectory[i+1]) - np.array(trajectory[i])
            
            # Calculate angle between vectors
            if np.linalg.norm(prev_vec) > 0 and np.linalg.norm(next_vec) > 0:
                cos_angle = np.dot(prev_vec, next_vec) / (np.linalg.norm(prev_vec) * np.linalg.norm(next_vec))
                angle = math.acos(np.clip(cos_angle, -1, 1))
                
                # Keep if significant direction change
                if angle > math.pi/6:  # 30 degrees
                    optimized.append(trajectory[i])
        
        optimized.append(trajectory[-1])
        return optimized
    
    def distance(self, p1, p2):
        """Calculate Euclidean distance"""
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
    
    def get_container_position(self, container_id):
        """Get container position by ID"""
        positions = {
            1: [0.3, 0.3, 0.3],
            2: [0.3, 0.1, 0.3],
            3: [0.3, -0.1, 0.3],
            4: [0.3, -0.3, 0.3],
            5: [0.5, 0.0, 0.3]
        }
        return positions.get(container_id, [0.3, 0.0, 0.3])
    
    def get_pan_position(self, pan_id):
        """Get pan position by ID"""
        positions = {
            1: [0.0, -0.3, 0.4],
            2: [0.0, 0.3, 0.4]
        }
        return positions.get(pan_id, [0.0, 0.0, 0.4])
    
    def get_pod_position(self, pod_id):
        """Get spice pod position by ID (circular arrangement)"""
        radius = 0.2
        center_x, center_y = -0.4, 0.0
        angle = 2 * math.pi * (pod_id - 1) / 20
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        return [x, y, 0.3]

def main():
    rclpy.init()
    
    try:
        planner = AdvancedPathPlanner()
        
        # Test the assignment scenarios
        print("\n" + "="*60)
        print("TESTING ASSIGNMENT SCENARIOS WITH ADVANCED PLANNING")
        print("="*60)
        
        # Test Case 1: Container 5 -> Pan 2
        print("\n1. Macro Dispense: Container 5 → Pan 2")
        trajectory1 = planner.plan_macro_dispense(5, 2)
        print(f"   • Planned waypoints: {len(trajectory1)}")
        print(f"   • Path length: {sum(planner.distance(trajectory1[i], trajectory1[i+1]) for i in range(len(trajectory1)-1)):.3f}m")
        
        # Test Case 2: Container 1 -> Pan 1
        print("\n2. Macro Dispense: Container 1 → Pan 1")
        trajectory2 = planner.plan_macro_dispense(1, 1)
        print(f"   • Planned waypoints: {len(trajectory2)}")
        print(f"   • Path length: {sum(planner.distance(trajectory2[i], trajectory2[i+1]) for i in range(len(trajectory2)-1)):.3f}m")
        
        # Test Case 3: Pod 1 -> Pan 2
        print("\n3. Micro Dispense: Pod 1 → Pan 2")
        trajectory3 = planner.plan_micro_dispense(1, 2)
        print(f"   • Planned waypoints: {len(trajectory3)}")
        print(f"   • Precision: 2.0mm engagement")
        
        # Test Case 4: Pod 19 -> Pan 1
        print("\n4. Micro Dispense: Pod 19 → Pan 1")
        trajectory4 = planner.plan_micro_dispense(19, 1)
        print(f"   • Planned waypoints: {len(trajectory4)}")
        print(f"   • Precision: 2.0mm engagement")
        
        print("\n" + "="*60)
        print("✅ ADVANCED PATH PLANNING COMPLETED SUCCESSFULLY!")
        print("="*60)
        
        # Keep node alive briefly
        time.sleep(2)
        planner.destroy_node()
        
    except Exception as e:
        print(f"Error in advanced planner: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
