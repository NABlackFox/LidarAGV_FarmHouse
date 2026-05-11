import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class StraightLineDetector(Node):
    def __init__(self):
        super().__init__('straight_line_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.prev_angle_left = None

    def fit_wall_angle(self, ranges, angles, angle_min, angle_max):
        mask = (
            (angles > angle_min) &
            (angles < angle_max) &
            np.isfinite(ranges)
        )
        valid_ranges = ranges[mask]
        valid_angles = angles[mask]

        if len(valid_ranges) < 5:
            return None

        # Polar → Cartesian
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)

        # Least squares line fit: y = mx + b
        A = np.vstack([x, np.ones(len(x))]).T
        m, _ = np.linalg.lstsq(A, y, rcond=None)[0]

        return np.arctan(m) * 180.0 / np.pi

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # ── LEFT SIDE ONLY (60° – 120°) ──────────────────────────────────────
        angle_left = self.fit_wall_angle(ranges, angles,
                                         angle_min=np.pi / 3,     #  60°
                                         angle_max=2 * np.pi / 3) # 120°

        if angle_left is None:
            self.get_logger().warn('LEFT: not enough points — is there a wall on the left?')
            return

        if self.prev_angle_left is None:
            self.get_logger().info(f'LEFT: first scan, angle={angle_left:.2f}° — waiting for next scan to compare...')
            self.prev_angle_left = angle_left
            return

        delta = abs(angle_left - self.prev_angle_left)

        if delta < 1.5:
            self.get_logger().info(f'STRAIGHT | angle={angle_left:.2f}°  delta={delta:.2f}°')
        else:
            self.get_logger().info(f'TURNING  | angle={angle_left:.2f}°  delta={delta:.2f}°')

        self.prev_angle_left = angle_left


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()