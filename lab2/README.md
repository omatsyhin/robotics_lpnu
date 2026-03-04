# Lab 2: Introduction to ROS2 and Simulation Environment

## Description

A simple ROS2 Python package showcasing the capabilities of Robot Operating System:
- Launching Gazebo simulation from ROS2 using launch files (`gazebo_ros2.launch.py`)
- Using `ros_gz_bridge` to connect Gazebo and ROS2 topics
- Controling robot motion via ROS2 publisher (`robot_controller.py`)
- Processing LiDAR data via ROS2 subscriber (`lidar_subscriber.py`)

## Building and Testing the Package

### Build the Package

```bash
# Launch the docker container
./scripts/cmd run

# Or if the container is already running open the new terminal
./scripts/cmd bash

# You should already be at /opt/ws when you enter the container

# Build the package
colcon build --package-select lab2

# Source the workspace (makes the package visible)
source install/setup.bash
```

### If You Need to Clean and Rebuild

```bash
# Clean workspace
rm -rf ./build ./install ./log

# Rebuild everything
colcon build

# Or rebuild only lab2
colcon build --packages-select lab2

# Source the workspace
source install/setup.bash
```

### Launch Everything

```bash
ros2 launch lab2 gazebo_ros2.launch.py
```

### Test the Controller (New Terminal)

```bash
./scripts/cmd bash
source /opt/ws/install/setup.bash

# Run the robot controller
ros2 run lab2 robot_controller
```

### Test the Subscriber (Another New Terminal)

```bash
./scripts/cmd bash
source /opt/ws/install/setup.bash

# Run the LiDAR subscriber
ros2 run lab2 lidar_subscriber
```

## Exploring ROS2 Tools

### List and Inspect Nodes

```bash
# See all running nodes
ros2 node list

# Get detailed info about a node
ros2 node info /robot_controller
ros2 node info /lidar_subscriber
```

### List and Inspect Topics

```bash
# ROS2 topics
ros2 topic list

# Gazebo topics (for comparison)
gz topic -l

# See who's publishing/subscribing
ros2 topic info /cmd_vel
ros2 topic info /lidar

# See message structure
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show sensor_msgs/msg/LaserScan

# View messages in real-time
ros2 topic echo /cmd_vel
ros2 topic echo /lidar --once
```

### Publish Manually

```bash
# Control robot from command line
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" -r 10

# Press Ctrl+C to stop
```

## Troubleshooting

### "Package not found"

```bash
# Make sure you built the package
cd /opt/ws
colcon build --packages-select lab2

# Source the workspace
source install/setup.bash
```

### Bridge not connecting

```bash
# Check topics on both sides
ros2 topic list | grep lidar
gz topic -l | grep lidar

# Check bridge node
ros2 node info /parameter_bridge
```

### RViz2 shows no data

- Fixed Frame must match sensor frame
- Topic must be `/lidar` (check spelling)
- LaserScan display must be enabled (checkbox)
- Check QoS settings if using modified nodes

### Persmission denied on Python files

```bash
chmod +x /opt/ws/src/code/lab2/lab2/*.py
```