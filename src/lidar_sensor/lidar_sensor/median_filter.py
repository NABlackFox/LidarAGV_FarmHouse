#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
from scipy.signal import medfilt


class MedianFilter(Node):

    def __init__(self):
        super().__init__('median_fillter')

        # ==========================================================
        # Canvas settings
        # ==========================================================

        # Pixel size
        self.frame_width = 400
        self.frame_height = 400

        # Real world size (meters)
        self.frame_real_width = 2.0
        self.frame_real_height = 2.0

        self.frame_width_half = self.frame_width // 2
        self.frame_height_half = self.frame_height // 2

        # ==========================================================
        # LiDAR settings
        # ==========================================================

        self.lidar_max_range = 2.0

        # ==========================================================
        # Blank canvas
        # ==========================================================

        self._blank = np.ones(
            (self.frame_height, self.frame_width, 3),
            dtype=np.uint8
        ) * 255

        # ==========================================================
        # ROS2 Publisher / Subscriber
        # ==========================================================

        self.bridge = CvBridge()
        self.pub_median_raw = self.create_publisher(
            Image,
            '/debug/median_filter',
            10
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info("Corridor center node started")

    # ==========================================================
    # Helper
    # ==========================================================

    def map_val(self, val, in_min, in_max, out_min, out_max):
        m = (out_max - out_min) / (in_max - in_min)
        return m * (val - in_min) + out_min

    # ==========================================================
    # Median Filter
    # ==========================================================

    def median_filter_scan(self, ranges, kernel_size=5):
        # Copy ranges
        clean_ranges = np.copy(ranges)
        # Find invalid values
        invalid = ~np.isfinite(clean_ranges)
        
        # Replace invalid values
        if np.any(~invalid):
            max_valid = np.nanmax(clean_ranges[~invalid])
        else:
            max_valid = self.lidar_max_range

        clean_ranges[invalid] = max_valid

        # Apply median filter
        filtered = medfilt(
            clean_ranges,
            kernel_size=kernel_size
        )

        return filtered

    # ==========================================================
    # Parse LiDAR scan -> pixel coordinates
    # ==========================================================

    def parse_scan(self, msg):
        ranges = np.array(
            msg.ranges,
            dtype=np.float32
        )
        # Generate matching angles
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        # ==========================================================
        # Apply Median Filter
        # ==========================================================
        ranges = self.median_filter_scan(
            ranges,
            kernel_size=5
        )
        # ==========================================================
        # Remove invalid points
        # ==========================================================

        valid = (
            np.isfinite(ranges)
            & (ranges > msg.range_min)
            & (ranges < msg.range_max)
            & (ranges <= self.lidar_max_range)
        )

        ranges = ranges[valid]
        angles = angles[valid]

        # ==========================================================
        # Polar -> Cartesian
        # ==========================================================

        X = ranges * np.cos(angles)
        Y = ranges * np.sin(angles)

        # ==========================================================
        # Cartesian -> Pixel
        # ==========================================================

        PY = self.map_val(
            X,
            -self.frame_real_height / 2,
             self.frame_real_height / 2,
            self.frame_height,
            0
        ).astype(np.int32)

        PX = self.map_val(
            Y,
            -self.frame_real_width / 2,
             self.frame_real_width / 2,
            self.frame_width,
            0
        ).astype(np.int32)

        return PX, PY

    # ==========================================================
    # Draw LiDAR bitmap
    # ==========================================================

    def draw_canvas(self, PX, PY):

        canvas = self._blank.copy()

        valid = (
            (PX >= 0) & (PX < self.frame_width)
            & (PY >= 0) & (PY < self.frame_height)
        )

        canvas[PY[valid], PX[valid]] = (0, 0, 0)

        # Draw robot center
        cv2.circle(
            canvas,
            (self.frame_width_half,self.frame_height_half),5,(255, 0, 0),-1)
        return canvas

    # ==========================================================
    # ROS2 Callback
    # ==========================================================

    def scan_callback(self, msg):

        PX, PY = self.parse_scan(msg)

        canvas = self.draw_canvas(PX, PY)

        self.pub_median_raw.publish(
            self.bridge.cv2_to_imgmsg(
                canvas,
                'bgr8'
            )
        )

    # ==========================================================
    # Main
    # ==========================================================


def main(args=None):

    rclpy.init(args=args)
    node = MedianFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()