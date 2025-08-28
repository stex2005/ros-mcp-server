#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Export TurtleBot3 model (default to burger if not set)
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:=burger}

echo "Starting ROS Bridge WebSocket server..."
echo "WebSocket server will be available at: ws://localhost:9090"

# Launch rosbridge websocket server in the background
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

# Wait a moment for rosbridge to start
sleep 3

echo "Starting TurtleBot3 simulation in Gazebo..."
echo "Model: $TURTLEBOT3_MODEL"
echo "Gazebo window should appear shortly"

# Launch TurtleBot3 Gazebo simulation in the background
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
GAZEBO_PID=$!

# Wait a moment for Gazebo to start
sleep 5

echo "Starting TurtleBot3 teleop..."
echo "You can control the robot using:"
echo "  - Arrow keys or WASD to move"
echo "  - Ctrl+C to stop"

# Launch TurtleBot3 keyboard teleop
ros2 run turtlebot3_teleop teleop_keyboard

# Cleanup when teleop exits
echo "Stopping TurtleBot3 simulation and rosbridge..."
kill $GAZEBO_PID
kill $ROSBRIDGE_PID

