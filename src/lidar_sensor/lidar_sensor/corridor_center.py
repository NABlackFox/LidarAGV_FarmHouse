#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy import pi
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2


class CorridorCenter(Node):

    def __init__(self):

        super().__init__('corridor_center')

        # ==========================================================
        # Canvas settings
        # ==========================================================

        # Pixel Unit
        self.frame_width = 400
        self.frame_height = 400

        # Meter Unit
        self.frame_real_width = 2.0 
        self.frame_real_height = 2.0

        self.frame_width_half = self.frame_width // 2
        self.frame_height_half = self.frame_height // 2

        # ==========================================================
        # Lidar range limit
        # ==========================================================

        # Limit for testing
        self.lidar_max_range = 2.0

        # ==========================================================
        # Lane / steering zone
        # ==========================================================

        # Assuming the hall is 0.7 wide
        self.lane_width_meter = 0.7
        self.half_lane_width_pixel = int((self.frame_width / self.frame_real_width)* 
                                         (self.lane_width_meter / 2.0))

        self.center_offset = 3          # deadband half-width in pixels
        self.center_min_DB = self.frame_width_half - self.center_offset
        self.center_max_DB = self.frame_width_half + self.center_offset

        # ==========================================================
        # Hough transform tuning
        # ==========================================================

        self.hough_threshold  = 15
        self.hough_min_length = 20
        self.hough_max_gap    = 30

        # Canny
        self.canny_min = 50
        self.canny_max = 200

        # ==========================================================
        # Moving-average filter length
        # ==========================================================

        self.filter_len = 10

        # ==========================================================
        # Blank canvas parameter
        # ==========================================================

        self._blank = (np.ones((self.frame_height, self.frame_width, 3),np.uint8) * 255)

        # ==========================================================
        # Publisher & Subcriber
        # ==========================================================

        self.bridge = CvBridge()

        self.pub_raw = self.create_publisher(
            Image, '/debug/lidar_raw', 10
        )
        self.pub_edges = self.create_publisher(
            Image, '/debug/lidar_edges', 10
        )
        self.pub_result = self.create_publisher(
            Image, '/debug/lidar_result', 10
        )

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # ==========================================================
        # Filter state
        # ==========================================================

        self._x_min_list = np.array([])
        self._x_max_list = np.array([])
        self._y_min_list = np.array([])
        self._y_max_list = np.array([])

        self.get_logger().info("Corridor center node started")

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
    # Parse scan -> pixel coords
    # ==========================================================

    def parse_scan(self, msg):
        angles = np.arange(
            msg.angle_min,
            msg.angle_max,
            msg.angle_increment
        )
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = angles[:len(ranges)]

        # Remove the result that out of limit range 
        valid = (
            np.isfinite(ranges)
            & (ranges > msg.range_min)
            & (ranges < msg.range_max)
            & (ranges <= self.lidar_max_range)
        )

        X = ranges[valid] * np.cos(angles[valid])
        Y = ranges[valid] * np.sin(angles[valid])

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

        # centre guide line
        cv2.line(
            canvas,
            (self.frame_width_half, 0),
            (self.frame_width_half, self.frame_height),
            (180, 180, 180), 1
        )

        # robot position dot
        cv2.circle(
            canvas,
            (self.frame_width_half, self.frame_height_half),
            5, (255, 0, 0), -1
        )

        return canvas

    # ==========================================================
    # Detect lines  (ported from original, with angle filter)
    # ==========================================================

    def detect_lines(self, canvas):

        gray  = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, self.canny_min, self.canny_max)
        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        linesP = cv2.HoughLinesP(
            edges, 1, pi / 180,
            self.hough_threshold,
            minLineLength=self.hough_min_length,
            maxLineGap=self.hough_max_gap
        )

        left_lines  = []   # each entry: [y_top, y_bot, x_top, x_bot, length, dist]
        right_lines = []
        result = canvas.copy()

        if linesP is None:
            return left_lines, right_lines, result, edges_color

        for line in linesP:
            x1, y1, x2, y2 = line[0]

            dx = x2 - x1
            dy = y2 - y1
            length = np.hypot(dx, dy)

            if length == 0:
                continue

            # ---- angle filter: reject near-horizontal lines ----
            angle_from_vertical = abs(dx) / length   # 0 = perfectly vertical
            if angle_from_vertical > 0.7:            # > ~45 deg from vertical
                cv2.line(result, (x1,y1), (x2,y2), (200,200,200), 1)
                continue

            # ---- centre clutter filter ----
            mid_x = (x1 + x2) / 2.0
            dist_from_center = abs(mid_x - self.frame_width_half)
            if dist_from_center < 10:
                cv2.line(result, (x1,y1), (x2,y2), (200,200,200), 1)
                continue

            # ---- rearrange so index-0 is the smaller y (top of image) ----
            if y1 > y2:
                y_top, y_bot = y2, y1
                x_top, x_bot = x2, x1
            else:
                y_top, y_bot = y1, y2
                x_top, x_bot = x1, x2

            entry = [y_top, y_bot, x_top, x_bot, length, dist_from_center]

            if mid_x < self.frame_width_half:
                left_lines.append(entry)
                cv2.line(result, (x1,y1), (x2,y2), (255,0,0), 1)
            else:
                right_lines.append(entry)
                cv2.line(result, (x1,y1), (x2,y2), (0,255,0), 1)

        return left_lines, right_lines, result, edges_color

    # ==========================================================
    # Pick best line  (longest y-span)
    # ==========================================================

    def best_line(self, lines):
        if not lines:
            return None
        # y-span = index[1] - index[0]
        idx = int(np.argmax([l[1] - l[0] for l in lines]))
        return lines[idx]   # full entry

    # ==========================================================
    # Main callback
    # ==========================================================

    def scan_callback(self, msg):

        # ---- raw bitmap ----
        PX, PY = self.parse_scan(msg)
        canvas  = self.draw_canvas(PX, PY)

        # Mapped point canvas
        self.pub_raw.publish(
            self.bridge.cv2_to_imgmsg(canvas, 'bgr8')
        )

        # ---- detect lines ----
        left_lines, right_lines, result, edges_color = (
            self.detect_lines(canvas)
        )

        # Detected edge canvas
        self.pub_edges.publish(
            self.bridge.cv2_to_imgmsg(edges_color, 'bgr8')
        )

        # ---- wall existence checks ----
        if not left_lines and not right_lines:
            self.get_logger().warn("NO WALLS")
            self.pub_result.publish(
                self.bridge.cv2_to_imgmsg(result, 'bgr8')
            )
            return

        left  = self.best_line(left_lines)  # [min_y, max_y, x_top, x_bottom]
        right = self.best_line(right_lines) # [min_y, max_y, x_top, x_bottom]

        # ---- compute mid-line (with half-lane fallback) ----
        if left and right:
            x_min_mid = (left[2] + right[2]) / 2.0
            y_min_mid = (left[0] + right[0]) / 2.0
            x_max_mid = (left[3] + right[3]) / 2.0
            y_max_mid = (left[1] + right[1]) / 2.0
            case = "LR"

        elif left and not right:
            x_min_mid = left[2] + self.half_lane_width_pixel
            y_min_mid = left[0]
            x_max_mid = left[3] + self.half_lane_width_pixel
            y_max_mid = left[1]
            case = "L_only"

        elif right and not left:
            x_min_mid = right[2] - self.half_lane_width_pixel
            y_min_mid = right[0]
            x_max_mid = right[3] - self.half_lane_width_pixel
            y_max_mid = right[1]
            case = "R_only"

        else:
            self.get_logger().warn("BEST LINE FAILED")
            return

        # ---- moving-average filter ----
        x_min_mid, self._x_min_list = self.moving_average(
            x_min_mid, self._x_min_list, self.filter_len)
        x_max_mid, self._x_max_list = self.moving_average(
            x_max_mid, self._x_max_list, self.filter_len)
        y_min_mid, self._y_min_list = self.moving_average(
            y_min_mid, self._y_min_list, self.filter_len)
        y_max_mid, self._y_max_list = self.moving_average(
            y_max_mid, self._y_max_list, self.filter_len)

        # ---- draw selected walls ----
        if left:
            cv2.line(result,
                     (left[2],  left[0]),
                     (left[3],  left[1]),
                     (255, 0, 0), 4)

        if right:
            cv2.line(result,
                     (right[2], right[0]),
                     (right[3], right[1]),
                     (0, 255, 0), 4)

        # ---- draw mid trajectory ----
        cv2.line(result,
                 (int(x_min_mid), int(y_min_mid)),
                 (int(x_max_mid), int(y_max_mid)),
                 (0, 0, 255), 4)

        # ---- alignment from top midpoint (front of robot view) ----
        offset = x_min_mid - self.frame_width_half

        if abs(offset) <= self.center_offset:
            status = "CENTER"
        elif offset < 0:
            status = "LEFT"
        else:
            status = "RIGHT"

        print(f"[{case}] {status} | offset = {offset:.1f}px")

        # ---- draw text ----
        cv2.putText(
            result,
            f"{status} ({offset:.1f}px)",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 0, 255), 2
        )

        # ---- publish ----
        self.pub_result.publish(
            self.bridge.cv2_to_imgmsg(result, 'bgr8')
        )


def main(args=None):
    rclpy.init(args=args)
    node = CorridorCenter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()