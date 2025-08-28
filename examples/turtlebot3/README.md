# Example - Turtlebot in Gazebo

**Difficulty**: Intermediate  

**Description**: Control a TurtleBot3 robot in the Gazebo simulator through the ROS MCP Server.  
This example demonstrates how MCP can interact with a mobile robot, send velocity commands, and receive odometry and sensor feedback in a physics-based environment.

---

## What You'll Learn

- Launching Gazebo with a robot and `ros2_control`
- Publishing velocity commands with `/cmd_vel`
- Reading odometry from `/odom`
- Exposing ROS topics/services via ROS Bridge WebSocket
- Connecting MCP to a more realistic robot than Turtlesim

## Prerequisites

Before starting this tutorial, make sure you have the following installed:

- **Docker**: [Install Docker](https://docs.docker.com/get-docker/)
- **Docker Compose**: Usually comes with Docker Desktop, or install separately
- **X11 forwarding** (for Linux, installed on host): `sudo apt-get install x11-apps`

## Features

- Automatic Docker setup with ROS2 Humble + Gazebo
- TurtleBot3 model (burger by default, can set waffle/waffle_pi)
- ROS Bridge WebSocket server on port 9090
- Keyboard teleop for manual driving
- MCP integration ready for natural language commands

## Quick Start

### 1. Build and Launch the Container

Navigate to the turtlesim example directory and build the Docker container:

```bash
cd examples/turtlebot3

docker compose build
```

### 2. Start the Container

Launch the turtlesim container:

```bash
docker-compose up -d
```

This will:
- Start the ROS Bridge WebSocket server (ws://localhost:9090)
- Launch TurtleBot3 in Gazebo
- Start keyboard teleop for manual control

## How to Control TurtleBot3

There are multiple ways to control the robot in this example:

### 1. MCP + LLM (Recommended)

The ROS MCP Server exposes TurtleBot3 through a WebSocket API.  
Connect your MCP-enabled LLM client to: `ws://localhost:9090`

Then issue natural language commands such as:

- "Move forward 1 meter"
- "Turn left 90 degrees"
- "Spin around"
- "What is the current odometry?"

The MCP server translates these instructions into ROS2 messages (`/cmd_vel`) or queries (`/odom`) automatically.  
This allows you to **control TurtleBot3 directly with language** instead of low-level ROS commands.

---

### 2. Keyboard Teleop (for manual testing)

You need first to access the container:
```bash
docker exec -it ros2-turtlebot3 bash
```

Once inside the container, you can manually launch turtle teleop to control the turtle:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```
This will allow you to use the keyboard to manually move the turtlebot3 in the environment.
Use:

- w/x : forward/backward
- a/d : rotate left/right
- space/s : stop
- CTRL+C : quit

### 3. ROS2 Topics (for direct control)

Publish velocity commands manually:
`ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10`

## Integration with MCP Server

Once turtlesim and rosbridge are running, you can connect the MCP server to control the turtle programmatically

Since it is running on the same machine, you can tell the LLM to connect to the robot on localhost. 


## Troubleshooting

### Display Issues (Linux)

If you encounter display issues on Linux, run:

```bash
xhost +local:docker
```

### Display Issues (macOS)

For macOS users, make sure XQuartz is running and configured:

```bash
# Start XQuartz
open -a XQuartz

# Allow connections from localhost
xhost +localhost
```

## Cleanup

To stop and remove the container:

```bash
docker-compose down
```

To remove the built image:

```bash
docker rmi ros-mcp-server_turtlesim
```

## Next Steps

Now that you have turtlesim running, you can:


1. **Try more complex commands** like drawing shapes or following paths
2. **Install ROS Locally** to add more nodes and services
3. **Explore other examples in this repository**

This example provides a foundation for understanding how the MCP server can interact with ROS2 systems, from simple simulators like turtlesim to complex robotic platforms. 
