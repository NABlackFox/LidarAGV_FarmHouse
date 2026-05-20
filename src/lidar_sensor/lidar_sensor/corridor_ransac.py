#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy import pi
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2


class CorridorCenterRANSAC(Node):

    def __init__(self):

        super().__init__('corridor_center_ransac')

        # ==========================================================
        # Canvas settings
        # ==========================================================

        self.frame_width = 400
        self.frame_height = 400

        self.frame_real_width = 2.0
        self.frame_real_height = 2.0

        self.frame_width_half = self.frame_width // 2
        self.frame_height_half = self.frame_height // 2

        # ==========================================================
        # Lidar range limit
        # ==========================================================

        self.lidar_max_range = 2.0

        # ==========================================================
        # Lane / steering zone
        # ==========================================================

        self.lane_width_meter = 0.7
        self.half_lane_width_pixel = int(
            (self.frame_width / self.frame_real_width) *
            (self.lane_width_meter / 2.0)
        )

        self.center_offset = 3          # deadband half-width in pixels
        self.center_min_DB = self.frame_width_half - self.center_offset
        self.center_max_DB = self.frame_width_half + self.center_offset

        # ==========================================================
        # RANSAC parameters
        # ==========================================================

        # How close a point must be to the fitted line to be an inlier (pixels)
        self.ransac_threshold = 5.0

        # Minimum inliers required to accept a line as a wall
        self.ransac_min_inliers = 10

        # Number of RANSAC iterations
        self.ransac_iterations = 100

        # Minimum y-span (pixels) for a wall line to be valid
        self.ransac_min_y_span = 30

        # Reject lines whose angle deviates more than this from vertical (radians)
        self.ransac_max_angle_from_vertical = pi / 4   # 45 degrees

        # ==========================================================
        # Moving-average filter length
        # ==========================================================

        self.filter_len = 10

        # ==========================================================
        # Blank canvas
        # ==========================================================

        self._blank = (np.ones(
            (self.frame_height, self.frame_width, 3), np.uint8) * 255)

        # ==========================================================
        # Publisher & Subscriber
        # ==========================================================

        self.bridge = CvBridge()

        self.pub_raw = self.create_publisher(Image, '/debug/lidar_raw', 10)
        self.pub_result = self.create_publisher(Image, '/debug/lidar_result', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ==========================================================
        # Filter state
        # ==========================================================

        self._x_min_list = np.array([])
        self._x_max_list = np.array([])
        self._y_min_list = np.array([])
        self._y_max_list = np.array([])

        self.get_logger().info("Corridor center RANSAC node started")

    # ==========================================================
    # Helpers
    # ==========================================================

    def map_val(self, val, in_min, in_max, out_min, out_max):
        m = (out_max - out_min) / (in_max - in_min)
        return m * (val - in_min) + out_min

    def moving_average(self, new_val, history, max_len):
        if len(history) < max_len:
            history = np.append(history, new_val)
        else:
            history = np.append(history[1:], new_val)
        return float(np.mean(history)), history

    # ==========================================================
    # Parse scan -> pixel coords (in REAL meters too for RANSAC)
    # ==========================================================

    def parse_scan(self, msg):
        angles = np.arange(
            msg.angle_min,
            msg.angle_max,
            msg.angle_increment
        )
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = angles[:len(ranges)]

        valid = (
            np.isfinite(ranges)
            & (ranges > msg.range_min)
            & (ranges < msg.range_max)
            & (ranges <= self.lidar_max_range)
        )

        X = ranges[valid] * np.cos(angles[valid])  # forward axis
        Y = ranges[valid] * np.sin(angles[valid])  # lateral axis

        PY = self.map_val(
            X,
            -self.frame_real_height / 2,
             self.frame_real_height / 2,
            self.frame_height, 0
        ).astype(int)

        PX = self.map_val(
            Y,
            -self.frame_real_width / 2,
             self.frame_real_width / 2,
            self.frame_width, 0
        ).astype(int)

        return PX, PY

    # ==========================================================
    # Draw raw lidar bitmap
    # ==========================================================

    def draw_canvas(self, PX, PY):
        canvas = self._blank.copy()

        valid = (
            (PX >= 0) & (PX < self.frame_width)
            & (PY >= 0) & (PY < self.frame_height)
        )
        canvas[PY[valid], PX[valid]] = (0, 0, 0)

        cv2.line(
            canvas,
            (self.frame_width_half, 0),
            (self.frame_width_half, self.frame_height),
            (180, 180, 180), 1
        )
        cv2.circle(
            canvas,
            (self.frame_width_half, self.frame_height_half),
            5, (255, 0, 0), -1
        )
        return canvas

    # ==========================================================
    # RANSAC line fit on a set of (px, py) points
    # Returns (slope, intercept, inlier_mask) in pixel space
    # Line model: px = slope * py + intercept  (x as function of y,
    # because walls are nearly vertical → small dy variation)
    # ==========================================================

    def ransac_fit_line(self, px, py):
        """
        Fit: px = a * py + b  (vertical-friendly parameterisation)
        Returns (a, b, inlier_mask) or None if fit fails.
        """
        n = len(px)
        if n < 2:
            return None

        best_inliers = None
        best_count = 0

        for _ in range(self.ransac_iterations):
            # Random sample of 2 points
            idx = np.random.choice(n, 2, replace=False)
            py1, px1 = py[idx[0]], px[idx[0]]
            py2, px2 = py[idx[1]], px[idx[1]]

            dy = py2 - py1
            if dy == 0:
                continue  # horizontal sample → skip

            a = (px2 - px1) / dy
            b = px1 - a * py1

            # Distance from each point to the line  px = a*py + b
            # => a*py - px + b = 0  (in form Ax + By + C = 0 with
            #    the role of x/y swapped to match our parameterisation)
            dist = np.abs(a * py - px + b) / np.sqrt(a ** 2 + 1)
            inliers = dist < self.ransac_threshold
            count = np.sum(inliers)

            if count > best_count:
                best_count = count
                best_inliers = inliers

        if best_count < self.ransac_min_inliers:
            return None

        # Refit using all inliers (least-squares)
        py_in = py[best_inliers]
        px_in = px[best_inliers]

        if len(py_in) < 2 or np.ptp(py_in) < self.ransac_min_y_span:
            return None

        coeffs = np.polyfit(py_in, px_in, 1)   # px = a*py + b
        a, b = coeffs[0], coeffs[1]

        # Angle filter: reject if too far from vertical
        angle = np.arctan(np.abs(a))   # angle from vertical (radians)
        if angle > self.ransac_max_angle_from_vertical:
            return None

        return a, b, best_inliers

    # ==========================================================
    # Segment points into left / right groups and run RANSAC
    # ==========================================================

    def detect_walls(self, PX, PY, canvas):
        result = canvas.copy()

        # Only keep points inside canvas
        valid = (
            (PX >= 0) & (PX < self.frame_width)
            & (PY >= 0) & (PY < self.frame_height)
        )
        px = PX[valid].astype(float)
        py = PY[valid].astype(float)

        if len(px) == 0:
            return None, None, result

        # Ignore points very close to centre (clutter / robot body)
        centre_mask = np.abs(px - self.frame_width_half) >= 10
        px = px[centre_mask]
        py = py[centre_mask]

        left_mask  = px < self.frame_width_half
        right_mask = px >= self.frame_width_half

        left_fit  = None
        right_fit = None

        # --- Left wall ---
        if np.sum(left_mask) >= self.ransac_min_inliers:
            fit = self.ransac_fit_line(px[left_mask], py[left_mask])
            if fit is not None:
                a, b, inliers = fit
                # Draw inlier points blue
                px_in = px[left_mask][inliers]
                py_in = py[left_mask][inliers]
                for xi, yi in zip(px_in.astype(int), py_in.astype(int)):
                    cv2.circle(result, (xi, yi), 2, (255, 100, 0), -1)

                # Compute line endpoints over y range
                y_top = int(py_in.min())
                y_bot = int(py_in.max())
                x_top = int(a * y_top + b)
                x_bot = int(a * y_bot + b)

                cv2.line(result, (x_top, y_top), (x_bot, y_bot), (255, 0, 0), 4)
                left_fit = (a, b, y_top, y_bot, x_top, x_bot)

        # --- Right wall ---
        if np.sum(right_mask) >= self.ransac_min_inliers:
            fit = self.ransac_fit_line(px[right_mask], py[right_mask])
            if fit is not None:
                a, b, inliers = fit
                px_in = px[right_mask][inliers]
                py_in = py[right_mask][inliers]
                for xi, yi in zip(px_in.astype(int), py_in.astype(int)):
                    cv2.circle(result, (xi, yi), 2, (0, 200, 100), -1)

                y_top = int(py_in.min())
                y_bot = int(py_in.max())
                x_top = int(a * y_top + b)
                x_bot = int(a * y_bot + b)

                cv2.line(result, (x_top, y_top), (x_bot, y_bot), (0, 200, 0), 4)
                right_fit = (a, b, y_top, y_bot, x_top, x_bot)

        return left_fit, right_fit, result

    # ==========================================================
    # Main callback
    # ==========================================================

    def scan_callback(self, msg):

        # --- raw bitmap ---
        PX, PY = self.parse_scan(msg)
        canvas  = self.draw_canvas(PX, PY)

        self.pub_raw.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8'))

        # --- detect walls via RANSAC ---
        left_fit, right_fit, result = self.detect_walls(PX, PY, canvas)

        if left_fit is None and right_fit is None:
            self.get_logger().warn("NO WALLS DETECTED")
            self.pub_result.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))
            return

        # --- compute mid-line (with half-lane fallback) ---
        # Each fit: (a, b, y_top, y_bot, x_top, x_bot)

        if left_fit and right_fit:
            x_min_mid = (left_fit[4] + right_fit[4]) / 2.0
            y_min_mid = (left_fit[2] + right_fit[2]) / 2.0
            x_max_mid = (left_fit[5] + right_fit[5]) / 2.0
            y_max_mid = (left_fit[3] + right_fit[3]) / 2.0
            case = "LR"

        elif left_fit and not right_fit:
            x_min_mid = left_fit[4] + self.half_lane_width_pixel
            y_min_mid = left_fit[2]
            x_max_mid = left_fit[5] + self.half_lane_width_pixel
            y_max_mid = left_fit[3]
            case = "L_only"

        else:  # right only
            x_min_mid = right_fit[4] - self.half_lane_width_pixel
            y_min_mid = right_fit[2]
            x_max_mid = right_fit[5] - self.half_lane_width_pixel
            y_max_mid = right_fit[3]
            case = "R_only"

        # --- moving-average filter ---
        x_min_mid, self._x_min_list = self.moving_average(
            x_min_mid, self._x_min_list, self.filter_len)
        x_max_mid, self._x_max_list = self.moving_average(
            x_max_mid, self._x_max_list, self.filter_len)
        y_min_mid, self._y_min_list = self.moving_average(
            y_min_mid, self._y_min_list, self.filter_len)
        y_max_mid, self._y_max_list = self.moving_average(
            y_max_mid, self._y_max_list, self.filter_len)

        # --- draw mid trajectory ---
        cv2.line(result,
                 (int(x_min_mid), int(y_min_mid)),
                 (int(x_max_mid), int(y_max_mid)),
                 (0, 0, 255), 4)

        # --- alignment from top midpoint ---
        offset = x_min_mid - self.frame_width_half

        if abs(offset) <= self.center_offset:
            status = "CENTER"
        elif offset < 0:
            status = "LEFT"
        else:
            status = "RIGHT"

        print(f"[{case}] {status} | offset = {offset:.1f}px")

        cv2.putText(
            result,
            f"{status} ({offset:.1f}px)",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 0, 255), 2
        )

        self.pub_result.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = CorridorCenterRANSAC()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()