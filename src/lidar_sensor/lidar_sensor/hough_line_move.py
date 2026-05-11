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

        # ── Image canvas size ───────────────────────────────────────────────
        # Think of this as the "paper" we draw on
        # frame_real_width/height = how many METERS the canvas covers
        # frame_width/height      = how many PIXELS the canvas has
        self.frame_width      = 300   # pixels (left-right)
        self.frame_height     = 600   # pixels (forward-backward)
        self.frame_real_width  = 2.0  # meters covered left-right
        self.frame_real_height = 4.0  # meters covered forward-backward

        self.frame_width_half  = self.frame_width  // 2  # = 150, center column
        self.frame_height_half = self.frame_height // 2  # = 300, center row

        # ── Lane width assumption ───────────────────────────────────────────
        # We assume the corridor is ~0.7m wide
        # This is used when ONE wall disappears — we estimate where the middle should be
        self.lane_width_meter      = 0.7
        self.half_lane_width_pixel = int(
            (self.frame_width / self.frame_real_width) * (self.lane_width_meter / 2.0)
        )

        # ── Steering deadband zones (in pixels) ────────────────────────────
        # If middle line x is within [center_min_DB, center_max_DB] → call it CENTER
        # Otherwise → SLIGHT LEFT or SLIGHT RIGHT
        self.center_min_DB = self.frame_width_half - 3
        self.center_max_DB = self.frame_width_half + 3
        self.x_mid_lane_MIN = self.frame_width_half - self.half_lane_width_pixel / 1.2
        self.x_mid_lane_MAX = self.frame_width_half + self.half_lane_width_pixel / 1.2

        # ── Hough line detection tuning ─────────────────────────────────────
        # houghThresh   = minimum number of points that must align to count as a line
        #                 lower = detects more lines (including noise), higher = stricter
        # houghMinLength = minimum line length in pixels to keep
        # houghMaxGap    = how big a gap between dots is still considered one line
        self.houghThresh    = 25
        self.houghMinLength = 10
        self.houghMaxGap    = 150

        # ── Stop zone geometry ──────────────────────────────────────────────
        robot_length   = 0.42
        robot_width    = 0.35
        AC_O           = 0.39   # lidar distance from robot back edge
        stop_dist      = 0.5    # how far ahead to check for obstacles
        B = (robot_length - AC_O,  robot_width / 2)
        D = (robot_length - AC_O, -robot_width / 2)
        T = (robot_length - AC_O + stop_dist, 0)
        self.B, self.D, self.T = B, D, T
        self.m1 = (T[1] - B[1]) / (T[0] - B[0])
        self.m2 = (T[1] - D[1]) / (T[0] - D[0])
        self.b1 = B[1] - self.m1 * B[0]
        self.b2 = D[1] - self.m2 * D[0]

        # ── Moving average buffers ──────────────────────────────────────────
        # Smooths out jittery line detections over last N frames
        self.line_list_length = 10
        self.x_min_list = np.array([])
        self.x_max_list = np.array([])

        # ── ROS2 publishers ─────────────────────────────────────────────────
        self.bridge = CvBridge()
        # We'll publish 3 image topics so you can see each stage in RViz2:
        self.pub_raw    = self.create_publisher(Image, '/debug/lidar_raw',    10)  # just the dots
        self.pub_edges  = self.create_publisher(Image, '/debug/lidar_edges',  10)  # after Canny
        self.pub_result = self.create_publisher(Image, '/debug/lidar_result', 10)  # with lines drawn

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("Started. View images with: ros2 run rqt_image_view rqt_image_view")

    # ── Helper: scale a value from one range to another ────────────────────
    def map_val(self, val, in_min, in_max, out_min, out_max):
        return ((out_max - out_min) / (in_max - in_min)) * (val - in_min) + out_min

    def in_front_stop_zone(self, x, y):
        if self.B[0] < x < self.T[0]:
            if self.D[1] < y < self.B[1]:
                if y < self.m1 * x + self.b1 and y > self.m2 * x + self.b2:
                    return True
        return False

    def moving_average(self, new_val, buf, size):
        if len(buf) < size:
            buf = np.append(buf, new_val)
        else:
            buf = np.append(buf[1:], new_val)
        return np.average(buf), buf

    def best_line(self, line_list):
        lengths = np.array([e[1] - e[0] for e in line_list])
        return line_list[np.argmax(lengths)]

    # ── Main callback ───────────────────────────────────────────────────────
    def scan_callback(self, msg):

        # ── STEP 1: raw distances + angles from lidar ───────────────────────
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = angles[:len(ranges)]  # guard off-by-one

        valid  = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        angles = angles[valid]
        ranges = ranges[valid]

        # ── STEP 2: polar → XY ─────────────────────────────────────────────
        # X = how far forward (positive = ahead of robot)
        # Y = how far sideways (positive = left, negative = right)
        X = ranges * np.cos(angles)
        Y = ranges * np.sin(angles)

        # ── STEP 3: check stop zone ─────────────────────────────────────────
        front_stop = any(self.in_front_stop_zone(x, y) for x, y in zip(X, Y))

        # ── STEP 4: XY → pixel coordinates ─────────────────────────────────
        # We're mapping the real-world meter grid onto the pixel canvas
        # X (forward) maps to PY (rows, top=far, bottom=close)
        # Y (sideways) maps to PX (columns, left=left, right=right)
        PY = self.map_val(X, -(self.frame_real_height/2), (self.frame_real_height/2), self.frame_height, 0)
        PX = self.map_val(Y, -(self.frame_real_width/2),  (self.frame_real_width/2),  self.frame_width,  0)

        # ── STEP 5: paint dots on blank white canvas ────────────────────────
        # Each lidar point becomes a black dot on white paper
        canvas = np.ones((self.frame_height, self.frame_width, 3), np.uint8) * 255
        px_arr = PX.astype(int)
        py_arr = PY.astype(int)
        in_bounds = (px_arr > 0) & (px_arr < self.frame_width) & \
                    (py_arr > 0) & (py_arr < self.frame_height)
        canvas[py_arr[in_bounds], px_arr[in_bounds]] = 0  # black dots

        # Draw the robot center as a blue circle
        cv2.circle(canvas, (self.frame_width_half, self.frame_height_half), 5, (255, 0, 0), -1)
        # Draw center column as a faint gray line (this is "straight ahead")
        cv2.line(canvas, (self.frame_width_half, 0), (self.frame_width_half, self.frame_height), (180,180,180), 1)

        # Publish and show STAGE 1: raw dots
        self.pub_raw.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8'))

        # ── STEP 6: Canny edge detection ────────────────────────────────────
        # Canny finds the EDGES of the black dot clusters
        # (sharp transition from white to black)
        gray  = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 200)
        # Convert back to color just for visualization
        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        # Publish and show STAGE 2: edges
        self.pub_edges.publish(self.bridge.cv2_to_imgmsg(edges_color, 'bgr8'))

        # ── STEP 7: Hough line detection ────────────────────────────────────
        # Finds straight lines through the edge pixels
        linesP = cv2.HoughLinesP(edges, 1, pi/180, self.houghThresh,
                                  minLineLength=self.houghMinLength,
                                  maxLineGap=self.houghMaxGap)

        left_lines, right_lines = [], []
        result = canvas.copy()  # we'll draw on this for the final view

        if linesP is not None:
            for line in linesP:
                x1, y1, x2, y2 = line[0]
                slope = None if x1 == x2 else (float(y2-y1)) / (float(x2-x1))

                # Skip horizontal lines — those aren't walls, just noise
                if slope is not None and not (1 <= abs(slope) <= 15):
                    cv2.line(result, (x1,y1), (x2,y2), (200,200,200), 1)  # gray = rejected
                    continue

                x_pos = x1 if slope is None else (x1+x2)/2.0
                entry = [min(y1,y2), max(y1,y2),
                         x1 if y1<y2 else x2,
                         x2 if y1<y2 else x1]

                if x_pos < self.frame_width_half:
                    left_lines.append(entry)
                    cv2.line(result, (x1,y1), (x2,y2), (255,0,0), 1)  # blue = left wall candidate
                else:
                    right_lines.append(entry)
                    cv2.line(result, (x1,y1), (x2,y2), (0,200,0), 1)  # green = right wall candidate

        # ── STEP 8: pick best line each side, compute middle ────────────────
        x_mid_L = x_mid_R = 0

        if left_lines:
            e = self.best_line(left_lines)
            x_mid_L = (e[2]+e[3])/2.0
            x_min_L, y_min_L, x_max_L, y_max_L = e[2], e[0], e[3], e[1]
            # Draw the CHOSEN left wall line thicker
            cv2.line(result, (x_min_L, y_min_L), (x_max_L, y_max_L), (255,0,0), 3)

        if right_lines:
            e = self.best_line(right_lines)
            x_mid_R = (e[2]+e[3])/2.0
            x_min_R, y_min_R, x_max_R, y_max_R = e[2], e[0], e[3], e[1]
            # Draw the CHOSEN right wall line thicker
            cv2.line(result, (x_min_R, y_min_R), (x_max_R, y_max_R), (0,200,0), 3)

        # ── STEP 9: compute where the middle line should be ─────────────────
        if x_mid_L and x_mid_R:
            x_min_MID = (x_min_L + x_min_R) / 2
            x_max_MID = (x_max_L + x_max_R) / 2
            y_min_MID = (y_min_L + y_min_R) / 2
            y_max_MID = (y_max_L + y_max_R) / 2
            case = "BOTH WALLS"
        elif x_mid_L:
            x_min_MID = x_min_L + self.half_lane_width_pixel
            x_max_MID = x_max_L + self.half_lane_width_pixel
            y_min_MID, y_max_MID = y_min_L, y_max_L
            case = "LEFT WALL ONLY"
        elif x_mid_R:
            x_min_MID = x_min_R - self.half_lane_width_pixel
            x_max_MID = x_max_R - self.half_lane_width_pixel
            y_min_MID, y_max_MID = y_min_R, y_max_R
            case = "RIGHT WALL ONLY"
        else:
            print("NO WALLS DETECTED")
            self.pub_result.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))
            return

        # ── STEP 10: smooth with moving average ─────────────────────────────
        x_min_MID, self.x_min_list = self.moving_average(x_min_MID, self.x_min_list, self.line_list_length)
        x_max_MID, self.x_max_list = self.moving_average(x_max_MID, self.x_max_list, self.line_list_length)

        # Draw the RED middle line
        cv2.line(result,
                 (int(x_min_MID), int(y_min_MID)),
                 (int(x_max_MID), int(y_max_MID)),
                 (0, 0, 255), 2)

        # ── STEP 11: decide status ──────────────────────────────────────────
        offset = x_min_MID - self.frame_width_half

        if front_stop:
            status = "!! OBSTACLE AHEAD !!"
            color  = (0, 0, 255)
        elif x_min_MID < self.center_min_DB:
            status = f">> SLIGHT LEFT  (offset {offset:+.1f}px)"
            color  = (0, 165, 255)
        elif x_min_MID > self.center_max_DB:
            status = f"<< SLIGHT RIGHT (offset {offset:+.1f}px)"
            color  = (0, 165, 255)
        else:
            status = f"== CENTER       (offset {offset:+.1f}px)"
            color  = (0, 200, 0)

        # Print to terminal
        print(f"{status}  [{case}]")

        # Burn the status text onto the result image
        cv2.putText(result, status, (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
        cv2.putText(result, case, (5, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 1, cv2.LINE_AA)

        # Publish STAGE 3: final result
        self.pub_result.publish(self.bridge.cv2_to_imgmsg(result, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ParallelLineDemo())
    rclpy.shutdown()

if __name__ == '__main__':
    main()