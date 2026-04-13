"""
ground_truth_publisher.py

Reads simulator ground truth from the /map -> /base_link TF transform and:
  - Publishes it as nav_msgs/Odometry on /gt/pose/odom  (for RViz / rosbag)
  - Publishes localization error on /pf/error (Float64MultiArray: [euclidean, heading_rad])
  - Writes a timestamped CSV with ground truth, PF estimate, and errors
  - Prints mean/std of euclidean error on shutdown (Ctrl-C)

CSV columns:
  time_sec, gt_x, gt_y, gt_theta, pf_x, pf_y, pf_theta,
  euclidean_error, heading_error_rad

Usage:
  ros2 run localization ground_truth_publisher
"""

import csv
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

import tf2_ros


class GroundTruthPublisher(Node):

    def __init__(self):
        super().__init__("ground_truth_publisher")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gt_pub = self.create_publisher(Odometry, "/gt/pose/odom", 1)
        self.err_pub = self.create_publisher(Float64MultiArray, "/pf/error", 1)

        self.pf_pose = None
        self.pf_sub = self.create_subscription(
            Odometry, "/pf/pose/odom", self._pf_callback, 1
        )

        self._euclidean_errors = []

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(os.getcwd(), f"pf_error_{timestamp}.csv")
        self._csv_file = open(csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "time_sec",
            "gt_x", "gt_y", "gt_theta",
            "pf_x", "pf_y", "pf_theta",
            "euclidean_error", "heading_error_rad",
        ])
        self.get_logger().info(f"Logging to {csv_path}")

        self.create_timer(0.02, self._publish)

        self.get_logger().info("GroundTruthPublisher ready – listening to /map -> /base_link")

    def _pf_callback(self, msg: Odometry):
        self.pf_pose = msg

    def _publish(self):
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        gt_x = t.x
        gt_y = t.y
        gt_theta = 2.0 * np.arctan2(q.z, q.w)

        now = self.get_clock().now().to_msg()
        time_sec = now.sec + now.nanosec * 1e-9

        gt_msg = Odometry()
        gt_msg.header.stamp = now
        gt_msg.header.frame_id = "map"
        gt_msg.child_frame_id = "base_link"
        gt_msg.pose.pose.position.x = gt_x
        gt_msg.pose.pose.position.y = gt_y
        gt_msg.pose.pose.orientation.z = q.z
        gt_msg.pose.pose.orientation.w = q.w
        self.gt_pub.publish(gt_msg)

        if self.pf_pose is None:
            return

        pf = self.pf_pose.pose.pose
        pf_x = pf.position.x
        pf_y = pf.position.y
        pf_theta = 2.0 * np.arctan2(pf.orientation.z, pf.orientation.w)

        dx = gt_x - pf_x
        dy = gt_y - pf_y

        euclidean = float(np.hypot(dx, dy))
        heading_err = float(np.arctan2(np.sin(gt_theta - pf_theta),
                                       np.cos(gt_theta - pf_theta)))

        self._euclidean_errors.append(euclidean)

        err_msg = Float64MultiArray()
        err_msg.data = [euclidean, heading_err]
        self.err_pub.publish(err_msg)

        self._csv_writer.writerow([
            f"{time_sec:.6f}",
            f"{gt_x:.6f}", f"{gt_y:.6f}", f"{gt_theta:.6f}",
            f"{pf_x:.6f}", f"{pf_y:.6f}", f"{pf_theta:.6f}",
            f"{euclidean:.6f}", f"{heading_err:.6f}",
        ])
        self._csv_file.flush()

    def _print_summary(self):
        errors = np.array(self._euclidean_errors)
        if len(errors) == 0:
            print("\n[ground_truth_publisher] No samples collected (PF never initialized?)")
        else:
            print(f"\n[ground_truth_publisher] Summary ({len(errors)} samples)")
            print(f"  Euclidean error  mean: {np.mean(errors):.4f} m")
            print(f"  Euclidean error  std:  {np.std(errors):.4f} m")

    def destroy_node(self):
        self._print_summary()
        if not self._csv_file.closed:
            self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
