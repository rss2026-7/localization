import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
import csv
import math

class ErrorLogger(Node):
    def __init__(self):
        super().__init__('error_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(Odometry, '/pf/pose/odom', self.cb, 10)
        self.csvfile = open('pf_error.csv', 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow(['timestamp', 'pf_x', 'pf_y', 'gt_x', 'gt_y', 'error'])

    def cb(self, msg):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            gt_x = t.transform.translation.x
            gt_y = t.transform.translation.y
            pf_x = msg.pose.pose.position.x
            pf_y = msg.pose.pose.position.y
            error = math.sqrt((pf_x - gt_x)**2 + (pf_y - gt_y)**2)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.writer.writerow([stamp, pf_x, pf_y, gt_x, gt_y, error])
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
