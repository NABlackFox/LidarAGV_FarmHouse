#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFilterLight(Node):
    def __init__(self):
        super().__init__('scan_filter_light')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')
        self.declare_parameter('min_range', 0.30)
        self.declare_parameter('max_range', 8.00)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            10
        )

        self.get_logger().info(
            f'Light scan filter started: {self.input_topic} -> {self.output_topic}, '
            f'range = {self.min_range:.2f}m to {self.max_range:.2f}m'
        )

    def scan_callback(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = max(msg.range_min, self.min_range)
        filtered.range_max = min(msg.range_max, self.max_range)
        filtered.intensities = msg.intensities

        new_ranges = []

        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                new_ranges.append(float('inf'))
            elif r < self.min_range:
                new_ranges.append(float('inf'))
            elif r > self.max_range:
                new_ranges.append(float('inf'))
            else:
                new_ranges.append(r)

        filtered.ranges = new_ranges
        self.publisher.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterLight()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
