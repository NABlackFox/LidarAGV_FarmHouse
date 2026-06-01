#!/usr/bin/env python3
"""
corridor_center.py
==================
AGV corridor-following node using LiDAR.

Overview
--------
The robot travels in a straight line between two tables.  Each table has
two legs arranged front-to-back (parallel to the travel direction).  The
LiDAR only sees a partial arc of each leg, producing small clusters of
points rather than continuous wall lines.

Because of this, the original Hough-line approach does not work well here.
Instead we use a three-stage image-processing pipeline that is similar in
structure to the original but replaces Canny + HoughLinesP with a simpler
and more robust method:

    parse_scan()          - same as original, polar -> pixel coords
    draw_canvas()         - same as original, dots on white canvas
    detect_leg_clusters() - NEW: thicken dots -> connectedComponents -> blob centroids
    build_virtual_walls() - NEW: average same-side centroids, offset by 20 cm
    scan_callback()       - same structure as original

Why connected components instead of Hough?
------------------------------------------
Hough line detection needs points arranged along a line.  A table leg
gives only a short arc of 5-15 points which Hough cannot reliably fit.
Connected components works at the blob level: it finds every separate
group of pixels in a binary image, regardless of shape.  This is a
standard OpenCV operation — no external libraries required.

Topic layout (unchanged from original)
---------------------------------------
  Sub : /scan                - sensor_msgs/LaserScan
  Pub : /debug/lidar_raw     - white canvas with raw LiDAR dots
        /debug/lidar_edges   - blob visualisation (replaces Canny edges)
        /debug/lidar_result  - final annotated result
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2


class CorridorCenter(Node):

    def __init__(self):
        super().__init__('corridor_center')

        # ==========================================================
        # Canvas settings  (unchanged)
        # ==========================================================

        self.frame_width  = 400          # pixels
        self.frame_height = 400          # pixels

        self.frame_real_width  = 2.0     # metres that the canvas width represents
        self.frame_real_height = 2.0     # metres that the canvas height represents

        self.frame_width_half  = self.frame_width  // 2
        self.frame_height_half = self.frame_height // 2

        # Conversion factor: pixels per metre
        self.px_per_m = self.frame_width / self.frame_real_width   # 200 px/m

        # ==========================================================
        # LiDAR range limit  (unchanged)
        # ==========================================================

        self.lidar_max_range = 2.0       # metres

        # ==========================================================
        # Blob detection parameters
        # These replace the Hough / Canny parameters from the original.
        # ==========================================================

        # Each raw LiDAR point is a single pixel.  We draw a filled
        # circle of this radius around every point so that nearby
        # points from the same leg merge into one connected blob.
        #
        # Rule of thumb:
        #   cluster_radius ≈ (expected gap between consecutive points
        #                     on the same leg arc) / 2
        # At 1 m range, a leg ~5 cm wide spans ~10 px.  A radius of
        # 8 px bridges small gaps without merging separate legs.
        self.cluster_radius = 8          # pixels

        # After merging, each blob's pixel area is checked.
        # Blobs outside this range are discarded:
        #   min_blob_area - removes single stray points and tiny noise
        #   max_blob_area - removes cases where two legs accidentally
        #                   merged (their combined area would be ~double)
        # Tune by watching /debug/lidar_edges: each leg should appear
        # as exactly one coloured blob.
        self.min_blob_area = 20          # pixels^2
        self.max_blob_area = 800         # pixels^2

        # ==========================================================
        # Virtual-wall offset
        # ==========================================================

        # The LiDAR detects leg centres, but we want to steer relative
        # to the table *edge*.  We shift the detected leg x-position
        # inward by this distance to get the virtual wall position.
        self.leg_to_edge_m  = 0.20
        self.leg_to_edge_px = int(self.px_per_m * self.leg_to_edge_m)

        # ==========================================================
        # Steering deadband  (unchanged)
        # ==========================================================

        self.center_offset = 3           # pixels — deadband half-width
        self.center_min_DB = self.frame_width_half - self.center_offset
        self.center_max_DB = self.frame_width_half + self.center_offset

        # Half-lane fallback: used when only one side is visible.
        # Set to half the expected corridor width in metres.
        self.half_lane_width_pixel = int(self.px_per_m * 0.35)

        # ==========================================================
        # Moving-average filter  (unchanged length)
        # ==========================================================

        self.filter_len  = 10
        self._mid_x_list = np.array([])

        # ==========================================================
        # Blank canvas  (unchanged)
        # ==========================================================

        self._blank = (
            np.ones((self.frame_height, self.frame_width, 3), np.uint8) * 255
        )

        # ==========================================================
        # ROS publishers / subscriber  (unchanged topics)
        # ==========================================================

        self.bridge = CvBridge()

        self.pub_raw    = self.create_publisher(Image, '/debug/lidar_raw',    10)
        self.pub_edges  = self.create_publisher(Image, '/debug/lidar_edges',  10)
        self.pub_result = self.create_publisher(Image, '/debug/lidar_result', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("Corridor center node started")

    # ==============================================================
    # Helpers  (unchanged)
    # ==============================================================

    def map_val(self, val, in_min, in_max, out_min, out_max):
        m = (out_max - out_min) / (in_max - in_min)
        return m * (val - in_min) + out_min

    def moving_average(self, new_val, history, max_len):
        if len(history) < max_len:
            history = np.append(history, new_val)
        else:
            history = np.append(history[1:], new_val)
        return float(np.mean(history)), history

    # ==============================================================
    # Step 1 - parse scan -> pixel coordinates  (unchanged)
    #
    # Converts each LiDAR (range, angle) reading into a (px, py)
    # pixel coordinate on the canvas.
    # ==============================================================

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

    # ==============================================================
    # Step 2 - draw raw LiDAR bitmap  (unchanged)
    #
    # Paints each valid LiDAR point as a black pixel on a white
    # canvas, plus the centre guide line and robot dot.
    # ==============================================================

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

    # ==============================================================
    # Step 3 - detect leg clusters  (replaces detect_lines)
    #
    # Original approach:
    #   Canny edge detection -> HoughLinesP -> filter by angle
    #   Works well for long continuous walls.
    #
    # New approach:
    #   Thicken dots -> binary image -> connectedComponentsWithStats
    #   Works well for short arcs from table legs.
    #
    # The key insight is that a table leg produces a small tight
    # cluster of LiDAR points, not a line.  Connected components
    # finds these clusters as separate blobs in a binary image.
    #
    # Sub-steps
    # ----------
    # 3a. Draw a filled circle of radius=cluster_radius around each
    #     LiDAR point on a black binary canvas.  Points that belong
    #     to the same leg are close enough that their circles overlap
    #     and merge into one connected white blob.
    #
    # 3b. cv2.connectedComponentsWithStats() scans the binary image
    #     and assigns a unique integer label to every separate white
    #     blob.  It also returns each blob's centroid (cx, cy) and
    #     area automatically — no line fitting needed.
    #
    # 3c. Filter blobs by area: too small = noise, too large = two
    #     legs accidentally merged.
    #
    # 3d. Split surviving blobs left/right by centroid x vs frame
    #     centre — identical logic to the original left/right line
    #     split in detect_lines().
    # ==============================================================

    def detect_leg_clusters(self, PX, PY, canvas):
        """
        Parameters
        ----------
        PX, PY  : pixel coordinate arrays from parse_scan()
        canvas  : raw canvas used as base for the debug image

        Returns
        -------
        left_clusters  : list of (cx, cy) blob centres left  of frame centre
        right_clusters : list of (cx, cy) blob centres right of frame centre
        cluster_canvas : colour-annotated debug image
        """

        cluster_canvas = canvas.copy()
        left_clusters  = []
        right_clusters = []

        # ---- 3a. build binary canvas with thickened dots ----
        # All-black image; we will paint white blobs onto it.
        binary = np.zeros((self.frame_height, self.frame_width), np.uint8)

        valid = (
            (PX >= 0) & (PX < self.frame_width)
            & (PY >= 0) & (PY < self.frame_height)
        )

        for px, py in zip(PX[valid], PY[valid]):
            # A filled white circle around every LiDAR point.
            # Points closer than 2 * cluster_radius will overlap,
            # causing them to merge into one blob per leg.
            cv2.circle(binary, (int(px), int(py)), self.cluster_radius, 255, -1)

        # ---- 3b. connected components ----
        # Returns:
        #   num_labels : total number of blobs found
        #                (label 0 is always the background)
        #   labels     : image where every pixel holds its blob label
        #   stats      : [num_labels x 5] array
        #                columns: x, y, width, height, area  per blob
        #   centroids  : [num_labels x 2] array — (cx, cy) per blob
        num_labels, labels, stats, centroids = (
            cv2.connectedComponentsWithStats(binary, connectivity=8)
        )

        # ---- 3c & 3d. filter and classify each blob ----
        for lbl in range(1, num_labels):    # label 0 = background, skip

            area = stats[lbl, cv2.CC_STAT_AREA]

            # Reject blobs outside the expected leg size range
            if area < self.min_blob_area or area > self.max_blob_area:
                continue

            cx = float(centroids[lbl, 0])
            cy = float(centroids[lbl, 1])

            # Left / right classification — same logic as original
            if cx < self.frame_width_half:
                colour = (255, 0, 0)        # blue  -> left leg
                left_clusters.append((cx, cy))
            else:
                colour = (0, 200, 0)        # green -> right leg
                right_clusters.append((cx, cy))

            # Paint the blob pixels on the debug canvas
            cluster_canvas[labels == lbl] = colour

            # Draw centroid marker
            cv2.circle(cluster_canvas, (int(cx), int(cy)), 6, colour, 2)

        return left_clusters, right_clusters, cluster_canvas

    # ==============================================================
    # Step 4 - build virtual walls from leg clusters
    #          (replaces best_line)
    #
    # Original: picked the single best (longest) Hough line per side.
    #
    # New: averages the x-positions of all detected leg clusters on
    # each side, then shifts inward by leg_to_edge_px.
    #
    # Why averaging works here
    # -------------------------
    # The two legs on each side are front-to-back, so they share
    # approximately the same x-position.  Averaging is therefore
    # equivalent to fitting a vertical line through them and is more
    # stable than using a single leg (noise on one leg only shifts
    # the average by half as much).
    #
    # Why the 20 cm offset
    # ---------------------
    # The LiDAR detects leg centres, but the robot should stay
    # parallel to the table *edge*, which is 20 cm outside the legs.
    # Shifting the detected x inward by this amount gives a reference
    # that represents the actual table edge.
    # ==============================================================

    def build_virtual_walls(self, left_clusters, right_clusters):
        """
        Returns
        -------
        left_wall_x  : float or None - pixel x of the left  virtual wall
        right_wall_x : float or None - pixel x of the right virtual wall
        case         : "LR" | "L_only" | "R_only" | "NONE"
        """

        left_wall_x  = None
        right_wall_x = None

        if left_clusters:
            avg_x = float(np.mean([c[0] for c in left_clusters]))
            left_wall_x = avg_x + self.leg_to_edge_px   # shift right (inward)

        if right_clusters:
            avg_x = float(np.mean([c[0] for c in right_clusters]))
            right_wall_x = avg_x - self.leg_to_edge_px  # shift left  (inward)

        if   left_wall_x is not None and right_wall_x is not None:
            case = "LR"
        elif left_wall_x is not None:
            case = "L_only"
        elif right_wall_x is not None:
            case = "R_only"
        else:
            case = "NONE"

        return left_wall_x, right_wall_x, case

    # ==============================================================
    # Main callback  (same structure as original)
    # ==============================================================

    def scan_callback(self, msg):

        # ---- Step 1: parse scan -> pixel coords ----
        PX, PY = self.parse_scan(msg)

        # ---- Step 2: draw raw bitmap ----
        canvas = self.draw_canvas(PX, PY)
        self.pub_raw.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8'))

        # ---- Step 3: detect leg clusters ----
        # Replaces: detect_lines(canvas)
        # Same input/output contract:
        #   in  -> canvas image + raw pixel arrays
        #   out -> left list, right list, debug image
        left_clusters, right_clusters, cluster_canvas = (
            self.detect_leg_clusters(PX, PY, canvas)
        )
        # Published on the same topic as Canny edges in the original
        self.pub_edges.publish(
            self.bridge.cv2_to_imgmsg(cluster_canvas, 'bgr8')
        )

        # ---- Step 4: build virtual walls ----
        # Replaces: best_line() + the LR/L_only/R_only midpoint block
        left_wall_x, right_wall_x, case = (
            self.build_virtual_walls(left_clusters, right_clusters)
        )

        if case == "NONE":
            self.get_logger().warn("NO LEGS DETECTED")
            self.pub_result.publish(
                self.bridge.cv2_to_imgmsg(canvas, 'bgr8')
            )
            return

        # ---- Step 5: compute corridor midpoint ----
        # Same three-case logic as original
        if case == "LR":
            mid_x = (left_wall_x + right_wall_x) / 2.0

        elif case == "L_only":
            mid_x = left_wall_x + self.half_lane_width_pixel

        else:   # R_only
            mid_x = right_wall_x - self.half_lane_width_pixel

        # ---- Step 6: moving-average filter  (unchanged) ----
        mid_x, self._mid_x_list = self.moving_average(
            mid_x, self._mid_x_list, self.filter_len
        )

        # ---- Step 7: draw result image ----
        result = cluster_canvas.copy()

        # Virtual wall lines — same visual role as the coloured Hough
        # lines in the original result image
        if left_wall_x is not None:
            cv2.line(result,
                     (int(left_wall_x), 0),
                     (int(left_wall_x), self.frame_height),
                     (255, 0, 0), 2)

        if right_wall_x is not None:
            cv2.line(result,
                     (int(right_wall_x), 0),
                     (int(right_wall_x), self.frame_height),
                     (0, 200, 0), 2)

        # Midpoint line — red, same as original trajectory line
        cv2.line(result,
                 (int(mid_x), 0),
                 (int(mid_x), self.frame_height),
                 (0, 0, 255), 2)

        # ---- Step 8: offset and steering status  (unchanged) ----
        offset = mid_x - self.frame_width_half

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

        # ---- Step 9: publish result  (unchanged) ----
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