#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from numpy import pi
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2


class ParallelLineDemo(Node):

    def __init__(self):
        super().__init__('parallel_line_demo')

        # ── Canvas size ─────────────────────────────────────────────────────
        # Asymmetric on purpose:
        #   Width  = 4m  → 2m each side of robot, enough for any corridor
        #   Height = 8m  → 4m ahead + 4m behind, fills screen as a corridor view
        # If 12m wide were used, the walls would be tiny slivers near center
        self.frame_width       = 300   # pixels (left-right)
        self.frame_height      = 600   # pixels (forward-backward)
        self.frame_real_width  = 4.0   # meters covered left-right  (2m each side)
        self.frame_real_height = 8.0   # meters covered forward-back (4m each side)
        self.frame_width_half  = self.frame_width  // 2  # center column = 150
        self.frame_height_half = self.frame_height // 2  # center row    = 300

        # ── Lidar range limit ────────────────────────────────────────────────
        # Lidar shoots up to 12m but 6m radius gives best results
        # Anything beyond 6m is discarded before processing
        self.lidar_max_range = 6.0  # meters

        # ── Adaptive lane width ─────────────────────────────────────────────
        # Learned from environment when BOTH walls are visible
        # Falls back to 0.7m default until we measure it ourselves
        self.learned_half_lane_px = int(
            (self.frame_width / self.frame_real_width) * (0.7 / 2.0)
        )
        # Per-wall memory: pixels from robot center to each wall
        # Held when that wall disappears at a crossroad
        self.dist_to_left_wall_px  = None
        self.dist_to_right_wall_px = None

        # ── Steering deadband ────────────────────────────────────────────────
        # If midline x is within ±3px of center → call it CENTER
        self.center_min_DB = self.frame_width_half - 3
        self.center_max_DB = self.frame_width_half + 3

        # ── Hough line detection tuning ──────────────────────────────────────
        self.houghThresh    = 25
        self.houghMinLength = 10
        self.houghMaxGap    = 150

        # ── Stop zone geometry ───────────────────────────────────────────────
        robot_length = 0.5
        robot_width  = 0.5
        AC_O         = 0.39  # lidar offset from robot back edge
        stop_dist    = 0.5   # how far ahead to check for obstacles
        B = (robot_length - AC_O,  robot_width / 2)
        D = (robot_length - AC_O, -robot_width / 2)
        T = (robot_length - AC_O + stop_dist, 0)
        self.B, self.D, self.T = B, D, T
        self.m1 = (T[1] - B[1]) / (T[0] - B[0])
        self.m2 = (T[1] - D[1]) / (T[0] - D[0])
        self.b1 = B[1] - self.m1 * B[0]
        self.b2 = D[1] - self.m2 * D[0]

        # ── Moving average buffers ───────────────────────────────────────────
        self.line_list_length = 10
        self.x_min_list = np.array([])
        self.x_max_list = np.array([])

        # ── ROS2 publishers ──────────────────────────────────────────────────
        self.bridge     = CvBridge()
        self.pub_raw    = self.create_publisher(Image, '/debug/lidar_raw',    10)
        self.pub_edges  = self.create_publisher(Image, '/debug/lidar_edges',  10)
        self.pub_result = self.create_publisher(Image, '/debug/lidar_result', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("Started. View: ros2 run rqt_image_view rqt_image_view")

    # ───────────────────────────────────────────────────────────────────────
    # Helpers
    # ───────────────────────────────────────────────────────────────────────

    def map_val(self, val, in_min, in_max, out_min, out_max):
        return ((out_max - out_min) / (in_max - in_min)) * (val - in_min) + out_min

    def in_front_stop_zone(self, x, y):
        if self.B[0] < x < self.T[0]:
            if self.D[1] < y < self.B[1]:
                if y < self.m1 * x + self.b1 and y > self.m2 * x + self.b2:
                    return True
        return False

    def moving_average(self, new_val, buf, size):
        buf = np.append(buf[-size+1:], new_val) if len(buf) >= size else np.append(buf, new_val)
        return np.average(buf), buf

    def best_line(self, line_list):
        lengths = np.array([e[1] - e[0] for e in line_list])
        return line_list[np.argmax(lengths)]

    # ───────────────────────────────────────────────────────────────────────
    # STEP 1+2: Parse lidar scan → XY points (hard capped at 6m)
    # ───────────────────────────────────────────────────────────────────────

    def parse_scan(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = angles[:len(ranges)]

        valid = (
            np.isfinite(ranges) &
            (ranges > msg.range_min) &
            (ranges < msg.range_max) &
            (ranges <= self.lidar_max_range)  # ← 6m hard cap
        )
        angles = angles[valid]
        ranges = ranges[valid]

        X = ranges * np.cos(angles)  # forward (+) / backward (-)
        Y = ranges * np.sin(angles)  # left (+) / right (-)
        return X, Y

    # ───────────────────────────────────────────────────────────────────────
    # STEP 3: Check stop zone
    # ───────────────────────────────────────────────────────────────────────

    def check_stop_zone(self, X, Y):
        return any(self.in_front_stop_zone(x, y) for x, y in zip(X, Y))

    # ───────────────────────────────────────────────────────────────────────
    # STEP 4+5: XY → canvas with lidar dots
    # ───────────────────────────────────────────────────────────────────────

    def draw_canvas(self, X, Y):
        PY = self.map_val(X, -(self.frame_real_height/2), (self.frame_real_height/2), self.frame_height, 0)
        PX = self.map_val(Y, -(self.frame_real_width/2),  (self.frame_real_width/2),  self.frame_width,  0)

        canvas = np.ones((self.frame_height, self.frame_width, 3), np.uint8) * 255
        px_arr, py_arr = PX.astype(int), PY.astype(int)
        in_bounds = (px_arr > 0) & (px_arr < self.frame_width) & \
                    (py_arr > 0) & (py_arr < self.frame_height)
        canvas[py_arr[in_bounds], px_arr[in_bounds]] = 0  # black dots

        # Robot center = blue dot, straight-ahead guide = faint gray line
        cv2.circle(canvas, (self.frame_width_half, self.frame_height_half), 5, (255, 0, 0), -1)
        cv2.line(canvas, (self.frame_width_half, 0), (self.frame_width_half, self.frame_height), (180,180,180), 1)
        return canvas

    # ───────────────────────────────────────────────────────────────────────
    # STEP 6+7: Canny + Hough → wall line candidates
    #   Blue  = left wall candidates
    #   Green = right wall candidates
    #   Gray  = rejected (too horizontal / not wall-like)
    # ───────────────────────────────────────────────────────────────────────

    def detect_lines(self, canvas):
        gray        = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
        edges       = cv2.Canny(gray, 50, 200)
        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        linesP = cv2.HoughLinesP(edges, 1, pi/180, self.houghThresh,
                                  minLineLength=self.houghMinLength,
                                  maxLineGap=self.houghMaxGap)

        left_lines, right_lines = [], []
        result = canvas.copy()

        if linesP is not None:
            for line in linesP:
                x1, y1, x2, y2 = line[0]
                slope = None if x1 == x2 else (float(y2-y1)) / (float(x2-x1))

                if slope is not None and not (1 <= abs(slope) <= 15):
                    cv2.line(result, (x1,y1), (x2,y2), (200,200,200), 1)  # gray = rejected
                    continue

                x_pos = x1 if slope is None else (x1+x2)/2.0
                entry = [min(y1,y2), max(y1,y2),
                         x1 if y1<y2 else x2,
                         x2 if y1<y2 else x1]

                if x_pos < self.frame_width_half:
                    left_lines.append(entry)
                    cv2.line(result, (x1,y1), (x2,y2), (255,0,0), 1)   # blue = left
                else:
                    right_lines.append(entry)
                    cv2.line(result, (x1,y1), (x2,y2), (0,200,0), 1)   # green = right

        return left_lines, right_lines, result, edges_color

    # ───────────────────────────────────────────────────────────────────────
    # STEP 8+9: Pick best wall lines, compute midline adaptively
    #   Chosen left wall  → thick blue  line
    #   Chosen right wall → thick green line
    # ───────────────────────────────────────────────────────────────────────

    def compute_midline(self, left_lines, right_lines, result):
        x_mid_L = x_mid_R = 0
        x_min_L = y_min_L = x_max_L = y_max_L = 0
        x_min_R = y_min_R = x_max_R = y_max_R = 0

        if left_lines:
            e = self.best_line(left_lines)
            x_min_L, y_min_L, x_max_L, y_max_L = e[2], e[0], e[3], e[1]
            x_mid_L = (x_min_L + x_max_L) / 2.0
            cv2.line(result, (x_min_L, y_min_L), (x_max_L, y_max_L), (255,0,0), 3)
            self.dist_to_left_wall_px = self.frame_width_half - x_mid_L

        if right_lines:
            e = self.best_line(right_lines)
            x_min_R, y_min_R, x_max_R, y_max_R = e[2], e[0], e[3], e[1]
            x_mid_R = (x_min_R + x_max_R) / 2.0
            cv2.line(result, (x_min_R, y_min_R), (x_max_R, y_max_R), (0,200,0), 3)
            self.dist_to_right_wall_px = x_mid_R - self.frame_width_half

        if x_mid_L and x_mid_R:
            self.learned_half_lane_px = int(abs(x_mid_R - x_mid_L) / 2)
            x_min_MID = (x_min_L + x_min_R) / 2
            x_max_MID = (x_max_L + x_max_R) / 2
            y_min_MID = (y_min_L + y_min_R) / 2
            y_max_MID = (y_max_L + y_max_R) / 2
            case = "BOTH WALLS"

        elif x_mid_L:
            dist = self.dist_to_left_wall_px if self.dist_to_left_wall_px is not None else self.learned_half_lane_px
            x_min_MID = x_min_L + dist
            x_max_MID = x_max_L + dist
            y_min_MID, y_max_MID = y_min_L, y_max_L
            case = "LEFT WALL ONLY"

        elif x_mid_R:
            dist = self.dist_to_right_wall_px if self.dist_to_right_wall_px is not None else self.learned_half_lane_px
            x_min_MID = x_min_R - dist
            x_max_MID = x_max_R - dist
            y_min_MID, y_max_MID = y_min_R, y_max_R
            case = "RIGHT WALL ONLY"

        else:
            return None, None, None, None, "NO WALLS"

        return x_min_MID, x_max_MID, y_min_MID, y_max_MID, case

    # ───────────────────────────────────────────────────────────────────────
    # STEP 10+11: Smooth + draw short RED midline + decide steering status
    #   Red short thick line = desired trajectory the robot should follow
    # ───────────────────────────────────────────────────────────────────────

    def compute_steering(self, x_min_MID, x_max_MID, y_min_MID, y_max_MID, front_stop, result):
        x_min_MID, self.x_min_list = self.moving_average(x_min_MID, self.x_min_list, self.line_list_length)
        x_max_MID, self.x_max_list = self.moving_average(x_max_MID, self.x_max_list, self.line_list_length)

        # Short red midline — 20% of canvas height centered on the detected corridor
        # Short enough to be readable, long enough to show direction
        mid_y_center = (y_min_MID + y_max_MID) / 2.0
        short_half   = self.frame_height * 0.10   # 10% each side
        mid_y_top    = int(mid_y_center - short_half)
        mid_y_bot    = int(mid_y_center + short_half)

        if y_max_MID != y_min_MID:
            t_top     = (mid_y_top - y_min_MID) / (y_max_MID - y_min_MID)
            t_bot     = (mid_y_bot - y_min_MID) / (y_max_MID - y_min_MID)
            mid_x_top = int(x_min_MID + t_top * (x_max_MID - x_min_MID))
            mid_x_bot = int(x_min_MID + t_bot * (x_max_MID - x_min_MID))
        else:
            mid_x_top = int(x_min_MID)
            mid_x_bot = int(x_max_MID)

        cv2.line(result, (mid_x_top, mid_y_top), (mid_x_bot, mid_y_bot), (0, 0, 255), 3)  # thick red

        offset = x_min_MID - self.frame_width_half

        if front_stop:
            status, color = "!! OBSTACLE AHEAD !!", (0, 0, 255)
        elif x_min_MID < self.center_min_DB:
            status, color = f">> SLIGHT LEFT  (offset {offset:+.1f}px)", (0, 165, 255)
        elif x_min_MID > self.center_max_DB:
            status, color = f"<< SLIGHT RIGHT (offset {offset:+.1f}px)", (0, 165, 255)
        else:
            status, color = f"== CENTER       (offset {offset:+.1f}px)", (0, 200, 0)

        return status, color

    # ───────────────────────────────────────────────────────────────────────
    # Main callback
    # ───────────────────────────────────────────────────────────────────────

    def scan_callback(self, msg):
        X, Y       = self.parse_scan(msg)
        front_stop = self.check_stop_zone(X, Y)
        canvas     = self.draw_canvas(X, Y)

        self.pub_raw.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8'))

        left_lines, right_lines, result, edges_color = self.detect_lines(canvas)

        self.pub_edges.publish(self.bridge.cv2_to_imgmsg(edges_color, 'bgr8'))

        x_min_MID, x_max_MID, y_min_MID, y_max_MID, case = \
            self.compute_midline(left_lines, right_lines, result)

        if case == "NO WALLS":
            print("NO WALLS DETECTED")
            self.pub_result.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))
            return

        status, color = self.compute_steering(
            x_min_MID, x_max_MID, y_min_MID, y_max_MID, front_stop, result
        )

        lane_w_m = (self.learned_half_lane_px * 2) * (self.frame_real_width / self.frame_width)
        print(f"{status}  [{case}]  lane≈{lane_w_m:.2f}m")

        cv2.putText(result, status,                   (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color,          1, cv2.LINE_AA)
        cv2.putText(result, case,                     (5, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0),        1, cv2.LINE_AA)
        cv2.putText(result, f"lane~{lane_w_m:.2f}m",  (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100,100,100), 1, cv2.LINE_AA)

        self.pub_result.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ParallelLineDemo())
    rclpy.shutdown()


if __name__ == '__main__':
    main()