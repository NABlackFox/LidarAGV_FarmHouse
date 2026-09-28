import time
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class SyncScanBridge(Node):

    def __init__(self):
        super().__init__('hector_sync_scan_bridge')

        self.declare_parameter(
            'scan_input',
            '/scan_hector_clean'
        )
        self.declare_parameter(
            'pose_input',
            '/hector_local/slam_out_pose'
        )
        self.declare_parameter(
            'scan_output',
            '/scan_slam'
        )
        self.declare_parameter(
            'odom_frame',
            'odom'
        )
        self.declare_parameter(
            'base_frame',
            'base_link'
        )
        self.declare_parameter(
            'release_delay',
            0.12
        )

        self.scan_input = self.get_parameter(
            'scan_input'
        ).value

        self.pose_input = self.get_parameter(
            'pose_input'
        ).value

        self.scan_output = self.get_parameter(
            'scan_output'
        ).value

        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.release_delay = float(
            self.get_parameter('release_delay').value
        )

        # Raw filtered scans waiting for the matching Hector pose.
        self.scan_buffer = deque(maxlen=100)

        # Hector-confirmed scans waiting briefly before publication.
        self.pending_scans = deque()

        self.tf_broadcaster = TransformBroadcaster(self)

        self.scan_pub = self.create_publisher(
            LaserScan,
            self.scan_output,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_input,
            self.scan_callback,
            50
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_input,
            self.pose_callback,
            50
        )

        # Non-blocking timer used to release scans after TF has had
        # enough time to propagate through DDS and enter TF buffers.
        self.release_timer = self.create_timer(
            0.01,
            self.release_pending_scans
        )

        self.matched_count = 0
        self.missed_count = 0
        self.published_count = 0

        self.get_logger().info(
            f'Sync bridge: {self.scan_input} + {self.pose_input} '
            f'-> TF {self.odom_frame}->{self.base_frame} '
            f'+ {self.scan_output}, '
            f'delay={self.release_delay:.3f}s'
        )

    @staticmethod
    def stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def scan_callback(self, msg):
        self.scan_buffer.append(msg)

    def pose_callback(self, pose_msg):
        pose_time = self.stamp_ns(
            pose_msg.header.stamp
        )

        matched_scan = None
        matched_index = None

        # First try exact timestamp matching.
        for i, scan in enumerate(self.scan_buffer):
            if self.stamp_ns(scan.header.stamp) == pose_time:
                matched_scan = scan
                matched_index = i
                break

        # Fallback tolerance: 5 ms.
        if matched_scan is None:
            best_error = 5_000_000

            for i, scan in enumerate(self.scan_buffer):
                error = abs(
                    self.stamp_ns(scan.header.stamp)
                    - pose_time
                )

                if error < best_error:
                    best_error = error
                    matched_scan = scan
                    matched_index = i

        if matched_scan is None:
            self.missed_count += 1
            return

        # Publish Hector pose as odom -> base_link first.
        tf_msg = TransformStamped()

        tf_msg.header.stamp = pose_msg.header.stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        tf_msg.transform.translation.x = (
            pose_msg.pose.position.x
        )

        tf_msg.transform.translation.y = (
            pose_msg.pose.position.y
        )

        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = (
            pose_msg.pose.orientation
        )

        self.tf_broadcaster.sendTransform(tf_msg)

        # Do NOT publish the scan immediately.
        # Store it and let the timer publish it later.
        release_time = (
            time.monotonic() + self.release_delay
        )

        self.pending_scans.append(
            (release_time, matched_scan)
        )

        self.matched_count += 1

        # Remove matched scan and all older scans.
        for _ in range(matched_index + 1):
            self.scan_buffer.popleft()

        if self.matched_count % 50 == 0:
            self.get_logger().info(
                f'Matched={self.matched_count}, '
                f'published={self.published_count}, '
                f'missed={self.missed_count}, '
                f'pending={len(self.pending_scans)}'
            )

    def release_pending_scans(self):
        now = time.monotonic()

        while self.pending_scans:
            release_time, scan = self.pending_scans[0]

            if release_time > now:
                break

            self.pending_scans.popleft()
            self.scan_pub.publish(scan)
            self.published_count += 1


def main(args=None):
    rclpy.init(args=args)

    node = SyncScanBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
