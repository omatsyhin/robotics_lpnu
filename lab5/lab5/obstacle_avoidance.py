"""Obstacle avoidance using Artificial Potential Fields (APF)

Subscribes:
    /scan  (sensor_msgs/LaserScan)
    /odom  (nav_msgs/Odometry)

Publishes:
    /cmd_vel (geometry_msgs/TwistStamped)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        # Parameters
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.goal_x = self.declare_parameter("goal_x", -2.5).value
        self.goal_y = self.declare_parameter("goal_y", -1.0).value

        # Internal state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.scan_data = None

        # Potential field gains
        self.k_att = 1.0
        self.k_rep = 0.4
        self.repulsive_range = 1.0  # meters

        # Velocity limits
        self.max_linear = 0.22
        self.max_angular = 1.5

        # ROS interfaces
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        # Publisher now uses TwistStamped
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_callback(self, msg):
        """Store latest LaserScan data"""
        self.scan_data = msg

    def odom_callback(self, msg):
        """Extract robot position and orientation from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.robot_yaw = yaw

    def compute_attractive_force(self):
        """Compute attractive force toward goal in world frame"""
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y

        return np.array([self.k_att * dx, self.k_att * dy])

    def compute_repulsive_force(self):
        """Compute repulsive force from obstacles using LaserScan"""
        if self.scan_data is None:
            return np.array([0.0, 0.0])

        force = np.array([0.0, 0.0])
        angle = self.scan_data.angle_min

        for r in self.scan_data.ranges:
            # Ignore invalid readings
            if math.isinf(r) or math.isnan(r):
                angle += self.scan_data.angle_increment
                continue

            # Filter out too small values to avoid instability
            if 0.05 < r < self.repulsive_range:
                mag = self.k_rep * (1.0 / r - 1.0 / self.repulsive_range) / (r * r)

                fx = -mag * math.cos(angle)
                fy = -mag * math.sin(angle)

                force += np.array([fx, fy])

            angle += self.scan_data.angle_increment

        # Transform from robot frame → world frame
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        rot = np.array([
            [cos_yaw, -sin_yaw],
            [sin_yaw,  cos_yaw]
        ])

        return rot @ force

    def control_loop(self):
        """Main control loop combining forces and publishing velocity"""
        F_att = self.compute_attractive_force()
        F_rep = self.compute_repulsive_force()
        F_total = F_att + F_rep

        # Desired heading
        desired_theta = math.atan2(F_total[1], F_total[0])

        # Heading error (normalize to [-pi, pi])
        angle_error = math.atan2(math.sin(desired_theta - self.robot_yaw),
                                math.cos(desired_theta - self.robot_yaw))

        # Distance to goal
        dist_to_goal = math.hypot(self.goal_x - self.robot_x,
                                self.goal_y - self.robot_y)

        # Create TwistStamped message
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Stop if goal reached
        if dist_to_goal < 0.1:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Goal reached at ({self.robot_x:.2f}, {self.robot_y:.2f})")
            return

        # Smoothed angular velocity (reduce overshoot)
        k_ang = 0.8  # smaller gain for angular correction
        msg.twist.angular.z = max(-self.max_angular,
                                min(self.max_angular, k_ang * angle_error))

        # Linear velocity: move forward even if heading is off, but scale down
        min_lin_speed = 0.05  # avoid being stuck
        lin_speed = self.max_linear * (1 - abs(angle_error) / math.pi)
        msg.twist.linear.x = max(min_lin_speed, min(self.max_linear, lin_speed))

        # Publish command
        self.cmd_pub.publish(msg)

        # Debug logs (optional)
        self.get_logger().debug(f"Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}), "
                                f"Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}), "
                                f"Heading error: {math.degrees(angle_error):.1f} deg, "
                                f"Linear: {msg.twist.linear.x:.2f}, Angular: {msg.twist.angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()