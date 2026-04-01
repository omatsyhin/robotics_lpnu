# Lab 5: Obstacle Avoidance

## Node Description

A ROS2 node which implements Artifial Potential Fields method for obstacle avoidance.

### Overview of the Algorithm

**The Artificial Potential Fields (APF)** algorithm is a reactive motion planning method used for mobile robots to navigate toward a goal while avoiding obstacles. The core idea is to treat the robot as a particle influenced by virtual forces:
- **Attractive forces** pull the robot toward the goal.
- **Repulsive forces** push the robot away from obstacles.

The robot computes the resultant force vector at each control cycle and translates it into linear and angular velocities. The APF method is simple and suitable for real-time control but may require careful tuning of gains to prevent oscillations near obstacles or goal overshoot.

Workflow of the algorithm:
1. Read the robot’s current position and orientation from odometry.
2. Read distance measurements from a LIDAR sensor.
3. Compute the attractive force toward the goal.
4. Compute the repulsive forces from nearby obstacles.
5. Sum the forces to obtain the total force vector.
6. Convert the total force into desired heading and linear speed.
7. Publish commands using a `TwistStamped` message.

### Key formulas

#### Attractive Force towards the Goal
The attractive force is proportional to the vector from the robot to the goal:

$$\mathbf{F}_{att} = k_{att} \begin{bmatrix} x_{goal} - x_{robot} \\ y_{goal} - y_{robot} \end{bmatrix}$$

The function that caclulates attractive force:
```python
def compute_attractive_force(self):
    """Compute attractive force toward goal in world frame"""
    dx = self.goal_x - self.robot_x
    dy = self.goal_y - self.robot_y
    return np.array([self.k_att * dx, self.k_att * dy])
```

#### Repulsive Force from Obstacles

The repulsive force depends on the distance to each obstacle. It is only active within a certain range `repulsive_range`:

$$\mathbf{F}_{rep} = \sum_{i=1}^{N} 
\begin{cases} 
k_{rep} \left( \frac{1}{d_i} - \frac{1}{d_0} \right) \frac{1}{d_i^2} \hat{\mathbf{u}}_i, & d_i < d_0 \\
0, & d_i \ge d_0
\end{cases}$$

Where:
- ${d_i}$ is the distance to the i-th obstacles point.
- ${d_0}$ is the influence range (`repulsive_range`).
- $\hat{\mathbf{u}}_i$ is the unit vector pointing from the obstacle to the robot.

The function that calculates repulsive force:

```python
def compute_repulsive_force(self):
    """Compute repulsive force from obstacles using LaserScan"""
    if self.scan_data is None:
        return np.array([0.0, 0.0])

    force = np.array([0.0, 0.0])
    angle = self.scan_data.angle_min

    for r in self.scan_data.ranges:
        if math.isinf(r) or math.isnan(r):
            angle += self.scan_data.angle_increment
            continue
        if 0.05 < r < self.repulsive_range:
            mag = self.k_rep * (1.0 / r - 1.0 / self.repulsive_range) / (r * r)
            fx = -mag * math.cos(angle)
            fy = -mag * math.sin(angle)
            force += np.array([fx, fy])
        angle += self.scan_data.angle_increment

    # Transform from robot frame to world frame
    cos_yaw = math.cos(self.robot_yaw)
    sin_yaw = math.sin(self.robot_yaw)
    rot = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
    return rot @ force
```

#### Computing Heading and Velocity

The total force is the sum of the attractive and repulsive forces:

$$\mathbf{F}_{total} = \mathbf{F}_{att} + \mathbf{F}_{rep}$$

```python
F_att = self.compute_attractive_force()
F_rep = self.compute_repulsive_force()
F_total = F_att + F_rep
```

The desired heading is:

$$\theta_{des} = \mathrm{atan2}(F_{total,y}, F_{total,x})$$

```python
desired_theta = math.atan2(F_total[1], F_total[0])
```

The angular velocty is a proportional controller based on heading error:

$$\omega = {k_0} \cdot (\theta_{des} - \theta_{robot})$$

```python
angle_error = math.atan2(math.sin(desired_theta - self.robot_yaw),
                                math.cos(desired_theta - self.robot_yaw))

k_ang = 0.8  
msg.twist.angular.z = max(-self.max_angular,
                        min(self.max_angular, k_ang * angle_error))
```

Linear velocity is scaled by how well the robot is aligned with the heading:

$$v = v_{max} \cdot \left( 1 - \frac{|\theta_{des} - \theta_{robot}|}{\pi} \right)$$

```python
min_lin_speed = 0.05
lin_speed = self.max_linear * (1 - abs(angle_error) / math.pi)
msg.twist.linear.x = max(min_lin_speed, min(self.max_linear, lin_speed))
```

### Pros and Cons of APF

#### Advantages:
- Requires minimal processing; suitable for real-time control on embedded platforms.
- Responds immediately to dynamic obstacles without global path planning.
- Can be adapted to different robots and sensor setups by tuning parameters.

#### Disadvantages:
- Can be adapted to different robots and sensor setups by tuning parameters.
- Robot may get trapped in configurations where the net force is zero but the goal is not reached.
- The algorithm does not plan a globally shortest or collision-free path.
- Requires careful tuning of attractive and repulsive gains, linear/angular velocity limits, and repulsive range.

Robot may get stuck in local minima or oscillate near narrow passages. Gain tuning (`k_att`, `k_rep`, `repulsive_range`) is crucial for stable behaviour.

## Setup

Launch the container and build the lab3 and lab5 packages:

```bash
cd /opt/ws
colcon build --packages-select lab3 lab5
source install/setup.bash
```

## Launch and test

```bash
ros2 launch lab5 obstacle_avoidance_bringup.launch.py
```

To change the desired goal position redefine these parameters in `obstacle_avoidance.py`:

```python
class ObstacleAvoidanceNode(Node):
    def __init__(self):
        # ...
        self.goal_x = self.declare_parameter("goal_x", -2.5).value
        self.goal_y = self.declare_parameter("goal_y", -1.0).value
        # ...
```