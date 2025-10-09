#!/usr/bin/env python3
"""
Terminal-based Real-time Visualizer for Posha Assignment
Shows animated robot movement in the terminal
"""

import time
import os
import math

class TerminalVisualizer:
    def __init__(self):
        self.workspace_width = 60
        self.workspace_height = 20
        self.robot_pos = [30, 10]  # Start in center
        
    def clear_screen(self):
        """Clear terminal screen"""
        os.system('clear')
    
    def draw_workspace(self, operation_name, step_name, progress):
        """Draw the workspace with robot and objects"""
        self.clear_screen()
        
        print("🤖 POSHA ROBOTICS - REAL-TIME VISUALIZATION")
        print("=" * self.workspace_width)
        print(f"Operation: {operation_name}")
        print(f"Step: {step_name}")
        print(f"Progress: [{('█' * int(progress * 40)):<40}] {int(progress * 100)}%")
        print()
        
        # Create workspace grid
        for y in range(self.workspace_height):
            line = ""
            for x in range(self.workspace_width):
                # Draw borders
                if x == 0 or x == self.workspace_width - 1 or y == 0 or y == self.workspace_height - 1:
                    line += "▓"
                # Draw robot
                elif abs(x - self.robot_pos[0]) <= 1 and abs(y - self.robot_pos[1]) <= 1:
                    line += "🤖"
                # Draw containers
                elif (abs(x - 45) <= 2 and abs(y - 5) <= 1):  # Container 5
                    line += "📦"
                elif (abs(x - 45) <= 2 and abs(y - 15) <= 1):  # Container 1
                    line += "📦"
                # Draw pans
                elif (abs(x - 15) <= 3 and abs(y - 15) <= 2):  # Pan 1
                    line += "🍳"
                elif (abs(x - 15) <= 3 and abs(y - 5) <= 2):   # Pan 2
                    line += "🍳"
                # Draw spice pods
                elif (abs(x - 5) <= 1 and abs(y - 7) <= 0):    # Pod 1
                    line += "🌶️"
                elif (abs(x - 5) <= 1 and abs(y - 13) <= 0):   # Pod 19
                    line += "🌶️"
                # Empty space
                else:
                    line += " "
            print(line)
        
        print()
        print("LEGEND: 🤖 Robot  📦 Container  🍳 Pan  🌶️ Spice Pod")
        print("=" * self.workspace_width)
    
    def animate_movement(self, start_pos, end_pos, operation_name, step_name, duration=2.0):
        """Animate robot movement between two points"""
        steps = 20
        for i in range(steps + 1):
            progress = i / steps
            # Calculate intermediate position
            current_x = start_pos[0] + (end_pos[0] - start_pos[0]) * progress
            current_y = start_pos[1] + (end_pos[1] - start_pos[1]) * progress
            
            self.robot_pos = [int(current_x), int(current_y)]
            self.draw_workspace(operation_name, step_name, progress)
            time.sleep(duration / steps)
    
    def run_complete_demo(self):
        """Run complete assignment demonstration"""
        operations = [
            {
                "name": "Container 5 → Pan 2",
                "steps": [
                    {"name": "Approaching Container 5", "start": [30, 10], "end": [45, 5]},
                    {"name": "Gripper Engagement", "start": [45, 5], "end": [45, 5]},
                    {"name": "Lifting Container", "start": [45, 5], "end": [45, 3]},
                    {"name": "Moving to Pan 2", "start": [45, 3], "end": [15, 3]},
                    {"name": "Dispensing Ingredients", "start": [15, 3], "end": [15, 5]},
                    {"name": "Returning Home", "start": [15, 5], "end": [30, 10]}
                ]
            },
            {
                "name": "Container 1 → Pan 1", 
                "steps": [
                    {"name": "Approaching Container 1", "start": [30, 10], "end": [45, 15]},
                    {"name": "Gripper Engagement", "start": [45, 15], "end": [45, 15]},
                    {"name": "Lifting Container", "start": [45, 15], "end": [45, 13]},
                    {"name": "Moving to Pan 1", "start": [45, 13], "end": [15, 13]},
                    {"name": "Dispensing Ingredients", "start": [15, 13], "end": [15, 15]},
                    {"name": "Returning Home", "start": [15, 15], "end": [30, 10]}
                ]
            },
            {
                "name": "Spice Pod 1 → Pan 2",
                "steps": [
                    {"name": "High-precision Approach", "start": [30, 10], "end": [5, 7]},
                    {"name": "Ultra-fine Engagement", "start": [5, 7], "end": [5, 7]},
                    {"name": "Lifting Spice Pod", "start": [5, 7], "end": [5, 5]},
                    {"name": "Moving to Pan 2", "start": [5, 5], "end": [15, 5]},
                    {"name": "Dispensing Spices", "start": [15, 5], "end": [15, 7]},
                    {"name": "Returning to Rack", "start": [15, 7], "end": [30, 10]}
                ]
            },
            {
                "name": "Spice Pod 19 → Pan 1", 
                "steps": [
                    {"name": "High-precision Approach", "start": [30, 10], "end": [5, 13]},
                    {"name": "Ultra-fine Engagement", "start": [5, 13], "end": [5, 13]},
                    {"name": "Lifting Spice Pod", "start": [5, 13], "end": [5, 11]},
                    {"name": "Moving to Pan 1", "start": [5, 11], "end": [15, 11]},
                    {"name": "Dispensing Spices", "start": [15, 11], "end": [15, 13]},
                    {"name": "Returning to Rack", "start": [15, 13], "end": [30, 10]}
                ]
            }
        ]
        
        print("🚀 Starting Terminal Visualization...")
        time.sleep(2)
        
        for op_idx, operation in enumerate(operations):
            op_progress = op_idx / len(operations)
            
            for step_idx, step in enumerate(operation["steps"]):
                step_progress = step_idx / len(operation["steps"])
                total_progress = op_progress + (step_progress / len(operations))
                
                self.animate_movement(
                    step["start"], 
                    step["end"], 
                    operation["name"], 
                    step["name"],
                    duration=1.5
                )
            
            print(f"✅ Completed: {operation['name']}")
            time.sleep(1)
        
        print("🎉 ALL OPERATIONS COMPLETED SUCCESSFULLY!")
        print("🤖 Posha Robotics Assignment - VISUALIZATION COMPLETE!")

def main():
    visualizer = TerminalVisualizer()
    visualizer.run_complete_demo()

if __name__ == "__main__":
    main()
