"""
Integrate /cmd_vel to estimate pose.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__("dead_reckoning")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("ground_truth_topic", "/odom")
        self.declare_parameter("path_dr_topic", "/path_dr")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("max_poses", 2000)

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        gt_topic = self.get_parameter("ground_truth_topic").value
        path_topic = self.get_parameter("path_dr_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.max_poses = int(self.get_parameter("max_poses").value)

        self.create_subscription(TwistStamped, cmd_topic, self.cmd_callback, 10)
        self.create_subscription(Odometry, gt_topic, self.gt_callback, 10)
        self.pub_path = self.create_publisher(Path, path_topic, 10)

        # State variables 
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_time = None

        # Ground truth 
        self.gt_x = None
        self.gt_y = None
        self.gt_theta = None

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

    def cmd_callback(self, msg: TwistStamped):
        current_time = self.get_clock().now().nanoseconds * 1e-9

        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Extract velocities
        v = msg.twist.linear.x
        w = msg.twist.angular.z

        # Dead reckoning integration 
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # Normalize theta 
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish path 
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y

        # Convert yaw to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose)

        # Limit path length
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses.pop(0)

        self.pub_path.publish(self.path_msg)

    def gt_callback(self, msg: Odometry):
        # Store ground truth pose 
        self.gt_x = msg.pose.pose.position.x
        self.gt_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        # yaw extraction from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.gt_theta = math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()