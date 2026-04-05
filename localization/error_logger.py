import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
import csv
import math
import numpy as np


class ErrorLogger(Node):
    def __init__(self):
        super().__init__('error_logger')
        self.declare_parameter('output_file', 'pf_error.csv')
        output_file = self.get_parameter('output_file').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(Odometry, '/pf/pose/odom', self.odom_cb, 10)
        self.particle_sub = self.create_subscription(PoseArray, '/pf/particles', self.particle_cb, 10)

        self.csvfile = open(output_file, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow([
            'timestamp',
            'pf_x', 'pf_y',
            'gt_x', 'gt_y',
            'error',
            'std_x', 'std_y', 'std_theta',
        ])
        self.get_logger().info(f'Logging to {output_file}')

        # Latest particle spread (updated by particle callback)
        self.std_x = float('nan')
        self.std_y = float('nan')
        self.std_theta = float('nan')

    def particle_cb(self, msg):
        if len(msg.poses) == 0:
            return
        xs = np.array([p.position.x for p in msg.poses])
        ys = np.array([p.position.y for p in msg.poses])
        # Recover yaw from quaternion (z, w only for 2D)
        thetas = np.array([2.0 * math.atan2(p.orientation.z, p.orientation.w)
                           for p in msg.poses])
        self.std_x = float(np.std(xs))
        self.std_y = float(np.std(ys))
        self.std_theta = float(np.std(thetas))

    def odom_cb(self, msg):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            gt_x = t.transform.translation.x
            gt_y = t.transform.translation.y
            pf_x = msg.pose.pose.position.x
            pf_y = msg.pose.pose.position.y
            error = math.sqrt((pf_x - gt_x)**2 + (pf_y - gt_y)**2)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.writer.writerow([
                stamp,
                pf_x, pf_y,
                gt_x, gt_y,
                error,
                self.std_x, self.std_y, self.std_theta,
            ])
        except Exception:
            pass

    def destroy_node(self):
        self.csvfile.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ErrorLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
