#!/bin/bash
echo "=== POSHA ROBOTICS - STANDALONE SIMULATION LAUNCH ==="

# Check if Gazebo is available
if ! command -v gazebo &> /dev/null; then
    echo "❌ ERROR: Gazebo not found!"
    echo "Please install Gazebo: sudo apt install gazebo11"
    exit 1
fi

# Check if world file exists
WORLD_FILE="src/posha_simulation/worlds/posha_kitchen.world"
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ ERROR: World file not found: $WORLD_FILE"
    exit 1
fi

echo "✅ Gazebo found: $(gazebo --version | head -1)"
echo "✅ World file found: $WORLD_FILE"
echo ""
echo "🚀 Launching Gazebo with Posha Kitchen World..."
echo "This will open a 3D visualization window with:"
echo "   - Kitchen table and workspace"
echo "   - 5 macro containers"
echo "   - 2 cooking pans" 
echo "   - Spice rack with pods"
echo "   - Robot base"
echo ""
echo "Press Ctrl+C to stop the simulation"
echo "=" * 50

# Launch Gazebo
gazebo --verbose "$WORLD_FILE"
