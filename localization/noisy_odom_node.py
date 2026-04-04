import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class NoisyOdomNode(Node):
    """
    Subscribes to /odom, injects Gaussian noise into the twist, and
    republishes on /odom_noisy for particle filter stress testing.

    To use: point the particle filter at odom_topic: "/odom_noisy" in
    params.yaml while this node is running. Swap back to "/odom" for
    normal operation.

    Noise parameters are tunable via ROS parameters:
        sigma_v   -- std dev on linear.x and linear.y (m/s), default 0.05
        sigma_w   -- std dev on angular.z (rad/s),          default 0.02
    """

    def __init__(self):
        super().__init__('noisy_odom_node')

        self.declare_parameter('sigma_v', 0.05)
        self.declare_parameter('sigma_w', 0.02)

        self.sigma_v = self.get_parameter('sigma_v').get_parameter_value().double_value
        self.sigma_w = self.get_parameter('sigma_w').get_parameter_value().double_value

        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.pub = self.create_publisher(Odometry, '/odom_noisy', 10)

        self.get_logger().info(
            f'NoisyOdomNode ready  sigma_v={self.sigma_v}  sigma_w={self.sigma_w}')

    def odom_callback(self, msg: Odometry):
        noisy = Odometry()
        noisy.header = msg.header
        noisy.child_frame_id = msg.child_frame_id

        # Copy pose component unchanged (particle filter uses twist only)
        noisy.pose = msg.pose

        # Inject independent Gaussian noise into each twist component
        noisy.twist.twist.linear.x = (
            msg.twist.twist.linear.x + np.random.normal(0.0, self.sigma_v))
        noisy.twist.twist.linear.y = (
            msg.twist.twist.linear.y + np.random.normal(0.0, self.sigma_v))
        noisy.twist.twist.angular.z = (
            msg.twist.twist.angular.z + np.random.normal(0.0, self.sigma_w))

        self.pub.publish(noisy)


def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
