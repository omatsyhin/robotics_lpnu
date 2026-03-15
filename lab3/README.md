# Lab 3: Moving Mobile Robots in Simulation

## Description

ROS2 Python package that consists of the following nodes:
- **Square path** (`square_path.py`): Odometry-based, move forward + turn 90°, repeat 4 times.
- **Circle path** (`circle_path.py`): Timed motion, constant linear + angular velocity for one full circle. 
- **Velocity publisher** (`velocity_publisher`): Publish constant (v, w), prints wheel speeds from `diff_drive_math`.
- **Odom path publisher** (`odom_path_publisher`): Subscribes to odometry, publishes `/path` for RViz2.
- **Figure-8 path** (`figure_8_path.py`): Timed motion, constant linear + angular velocity for two full circles in figure-8.

There are also two robots included for testing the nodes:
- **TurtleBot3**: built-in ROS2 robot that comes with `ros-jazzy-turtlebot3` and `ros-jazzy-turtlebot3-simulations` packages.
- **Custom 4-wheel differential drive robot** from the previous labs (see [Lab 1](../lab1/README.md) and [Lab 2](../lab2/README.md)) 

## Launch

Launch the Docker container:
```bash
./scripts/cmd run
```

Build the package:
```bash
cd /opt/ws
colcon build --packages-select lab3
source install/setup.bash
```

**Custom 4-wheel robot:**
```bash
ros2 launch lab3 bringup.launch.py
```

**TurtleBot3:** 
```bash
ros2 launch lab3 turtlebot3_room_bringup.launch.py
```

## Run path nodes

To run the path nodes on 4-wheel robot you can use the following commands:
```bash
ros2 run lab3 square_path
ros2 run lab3 circle_path
ros2 run lab3 figure_8_path
```

> [!IMPORTANT]
> See [TurtleBot3 README](turtlebot3/README.md) for launch and path commands for TurtleBot3.

## Troubleshooting

**RViz shows no path:** Fixed Frame must be `odom`. Add Path display, topic `/path`.

## Code Structure

```
lab3/
├── lab3/
│   ├── diff_drive_math.py
│   ├── velocity_publisher.py
│   ├── odom_path_publisher.py
│   ├── square_path.py
│   ├── circle_path.py
│   └── figure_8_path.py       
├── launch/
│   ├── gazebo.launch.py
│   ├── bringup.launch.py
│   ├── turtlebot3_room.launch.py
│   └── turtlebot3_room_bringup.launch.py
├── turtlebot3/
│   ├── worlds/room.sdf
│   ├── urdf/README.md
│   ├── xacro/simple_diff_drive.urdf.xacro
│   └── README.md
├── rviz/
│   └── trajectory.rviz
└── worlds/
    └── robot.sdf
```
