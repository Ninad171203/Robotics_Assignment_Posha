#!/bin/bash
echo "=== POSHA ROBOTICS - TEXT-BASED VISUALIZATION ==="
echo "Since RViz may not be available, using advanced text visualization..."
echo ""

cd ~/posha_ros2_ws
python3 terminal_visualizer.py
