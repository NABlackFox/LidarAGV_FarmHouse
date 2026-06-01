#!/usr/bin/env python3
"""
test_table_leg.py
==================
AGV corridor-following node using LiDAR.

Strategy
--------
Instead of detecting continuous walls (Hough lines), this node detects
discrete table *legs* using DBSCAN clustering, then builds a virtual wall
line on each side by averaging the x-positions of same-side leg clusters.
A 20 cm inward offset is applied so the virtual wall represents the table
edge rather than the leg centre.  The robot steers toward the midpoint
of the two virtual walls.

Parallel-travel guarantee
--------------------------
Because both legs on one side are front-to-back (aligned with the travel
direction), averaging their x-positions gives a stable lateral reference
that is inherently parallel to the travel axis.  Noise on one leg only
pulls the average slightly; it does not rotate the reference line.

Topic layout (unchanged from original)
---------------------------------------
  Sub : /scan          (sensor_msgs/LaserScan)
  Pub : /debug/lidar_raw     - white canvas with raw LiDAR dots
        /debug/lidar_edges   - DBSCAN cluster visualisation
        /debug/lidar_result  - final annotated result
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
from sklearn.cluster import DBSCAN

class CorridorCenterDBScan(Node):

    def __init__(self):
        super().__init__('corridor_center_db_scan')

        # ==========================================================
        # Canvas settings  (same as original)
        # ==========================================================

        self.frame_width  = 400
        self.frame_height = 400

        self.frame_real_width  = 2.0   # metres represented by canvas width
        self.frame_real_height = 2.0   # metres represented by canvas height

        self.frame_width_half  = self.frame_width  // 2
        self.frame_height_half = self.frame_height // 2

        # ==========================================================
        # LiDAR range limit
        # ==========================================================

        self.lidar_max_range = 2.0     # metres

        # ==========================================================
        # Virtual-wall offset
        # ==========================================================

        # Physical distance from leg centre to table edge (metres).
        # The virtual wall is placed this far inward from each leg cluster.
        self.leg_to_edge_m = 0.20

        # Convert to pixels once
        self.leg_to_edge_px = int(
            (self.frame_width / self.frame_real_width) * self.leg_to_edge_m
        )

        # ==========================================================
        # Steering deadband  (same as original)
        # ==========================================================

        self.center_offset  = 3        # half-width in pixels
        self.center_min_DB  = self.frame_width_half - self.center_offset
        self.center_max_DB  = self.frame_width_half + self.center_offset

        # Half-lane fallback: used when only one side is visible.
        # Set this to half the expected corridor width in metres (e.g. 0.35 m).
        self.half_lane_width_pixel = int(
            (self.frame_width / self.frame_real_width) * 0.35
        )

        # ==========================================================
        # DBSCAN parameters
        # ==========================================================

        # eps   : max pixel distance between two points to be neighbours.
        #         A table leg ~5 cm wide at 1 m range → ~10 px.  Use 12.
        # min_s : minimum points to form a core cluster.
        #         We only see a partial arc of each leg, so keep this small.
        self.dbscan_eps       = 12
        self.dbscan_min_pts   = 3

        # Minimum cluster size to be counted as a leg (filters tiny noise).
        self.min_cluster_pts  = 4

        # ==========================================================
        # Moving-average filter  (same length as original)
        # ==========================================================

        self.filter_len = 10
        self._mid_x_list = np.array([])

        # ==========================================================
        # Blank canvas  (same as original)
        # ==========================================================

        self._blank = (
            np.ones((self.frame_height, self.frame_width, 3), np.uint8) * 255
        )

        # ==========================================================
        # ROS publishers / subscriber  (same topics as original)
        # ==========================================================

        self.bridge = CvBridge()

        self.pub_raw   = self.create_publisher(Image, '/debug/lidar_raw',    10)
        self.pub_edges = self.create_publisher(Image, '/debug/lidar_edges',  10)
        self.pub_result= self.create_publisher(Image, '/debug/lidar_result', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("Corridor center node started (DBSCAN mode)")

    # ==============================================================
    # Helpers  (identical to original)
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
    # Step 1 – parse scan → pixel coordinates
    # (identical to original parse_scan)
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
    # Step 2 – draw raw LiDAR bitmap
    # (identical to original draw_canvas)
    # ==============================================================

    def draw_canvas(self, PX, PY):
        canvas = self._blank.copy()

        valid = (
            (PX >= 0) & (PX < self.frame_width)
            & (PY >= 0) & (PY < self.frame_height)
        )
        canvas[PY[valid], PX[valid]] = (0, 0, 0)

        # Centre guide line
        cv2.line(
            canvas,
            (self.frame_width_half, 0),
            (self.frame_width_half, self.frame_height),
            (180, 180, 180), 1
        )

        # Robot position dot
        cv2.circle(
            canvas,
            (self.frame_width_half, self.frame_height_half),
            5, (255, 0, 0), -1
        )

        return canvas

    # ==============================================================
    # Step 3 – detect leg clusters
    #
    # This replaces detect_lines() from the original.
    # Instead of Canny + HoughLinesP, we use DBSCAN on the raw
    # pixel coordinates.  The output format mirrors the original:
    #   left_clusters  – list of (cx, cy) for clusters left of centre
    #   right_clusters – list of (cx, cy) for clusters right of centre
    #   cluster_canvas – debug image (replaces edges image)
    # ==============================================================

    def detect_leg_clusters(self, PX, PY, canvas):
        """
        Run DBSCAN on LiDAR pixel points and split resulting clusters
        into left-side and right-side leg candidates.

        Parameters
        ----------
        PX, PY       : pixel coordinate arrays from parse_scan()
        canvas       : raw canvas (used as base for the debug image)

        Returns
        -------
        left_clusters  : list of (cx, cy) — clusters whose centre x < frame centre
        right_clusters : list of (cx, cy) — clusters whose centre x >= frame centre
        cluster_canvas : annotated debug image
        """

        cluster_canvas = canvas.copy()
        left_clusters  = []
        right_clusters = []

        # ---- guard: need at least min_pts points to cluster ----
        valid_mask = (
            (PX >= 0) & (PX < self.frame_width)
            & (PY >= 0) & (PY < self.frame_height)
        )
        PX_v = PX[valid_mask]
        PY_v = PY[valid_mask]

        if len(PX_v) < self.dbscan_min_pts:
            return left_clusters, right_clusters, cluster_canvas

        # ---- DBSCAN clustering ----
        # Stack into (N, 2) array of [px_x, px_y] points
        points = np.column_stack([PX_v, PY_v])

        labels = DBSCAN(
            eps=self.dbscan_eps,
            min_samples=self.dbscan_min_pts
        ).fit_predict(points)

        # ---- process each cluster ----
        for label in set(labels):

            if label == -1:
                # Noise — draw as small grey dots
                noise_mask = labels == -1
                for px, py in zip(PX_v[noise_mask], PY_v[noise_mask]):
                    cv2.circle(cluster_canvas, (px, py), 1, (200, 200, 200), -1)
                continue

            cluster_mask = labels == label

            # Skip clusters that are too small (sensor noise)
            if np.sum(cluster_mask) < self.min_cluster_pts:
                continue

            cx = float(np.mean(PX_v[cluster_mask]))
            cy = float(np.mean(PY_v[cluster_mask]))

            # ---- classify left / right by cx vs frame centre ----
            if cx < self.frame_width_half:
                left_clusters.append((cx, cy))
                colour = (255, 0, 0)       # blue  → left
            else:
                right_clusters.append((cx, cy))
                colour = (0, 200, 0)       # green → right

            # Draw cluster points
            for px, py in zip(PX_v[cluster_mask], PY_v[cluster_mask]):
                cv2.circle(cluster_canvas, (px, py), 2, colour, -1)

            # Draw centroid
            cv2.circle(cluster_canvas, (int(cx), int(cy)), 6, colour, 2)
            cv2.putText(
                cluster_canvas,
                f"L{label}",
                (int(cx) + 7, int(cy) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, colour, 1
            )

        return left_clusters, right_clusters, cluster_canvas

    # ==============================================================
    # Step 4 – build virtual walls from leg clusters
    #
    # Replaces best_line() from the original.
    # For each side, average the x-positions of all detected leg
    # clusters, then offset inward by leg_to_edge_px.
    # ==============================================================

    def build_virtual_walls(self, left_clusters, right_clusters):
        """
        Convert leg cluster centroids into virtual wall x-positions.

        The table legs are front-to-back (parallel to travel), so all
        legs on the same side share approximately the same x.  Averaging
        their cx values gives a robust lateral reference.

        Returns
        -------
        left_wall_x  : float or None — pixel x of the left virtual wall
        right_wall_x : float or None — pixel x of the right virtual wall
        case         : str describing which sides were found
        """

        left_wall_x  = None
        right_wall_x = None

        if left_clusters:
            # Average x across however many left legs are visible
            raw_x = float(np.mean([c[0] for c in left_clusters]))
            # Offset inward (rightward) by the leg-to-edge distance
            left_wall_x = raw_x + self.leg_to_edge_px

        if right_clusters:
            raw_x = float(np.mean([c[0] for c in right_clusters]))
            # Offset inward (leftward)
            right_wall_x = raw_x - self.leg_to_edge_px

        # ---- determine case label (mirrors original LR / L_only / R_only) ----
        if left_wall_x is not None and right_wall_x is not None:
            case = "LR"
        elif left_wall_x is not None:
            case = "L_only"
        elif right_wall_x is not None:
            case = "R_only"
        else:
            case = "NONE"

        return left_wall_x, right_wall_x, case

    # ==============================================================
    # Main callback
    # ==============================================================

    def scan_callback(self, msg):

        # ---- Step 1: parse scan → pixel coords ----
        PX, PY = self.parse_scan(msg)

        # ---- Step 2: draw raw bitmap ----
        canvas = self.draw_canvas(PX, PY)
        self.pub_raw.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8'))

        # ---- Step 3: detect leg clusters (replaces detect_lines) ----
        left_clusters, right_clusters, cluster_canvas = (
            self.detect_leg_clusters(PX, PY, canvas)
        )
        self.pub_edges.publish(
            self.bridge.cv2_to_imgmsg(cluster_canvas, 'bgr8')
        )

        # ---- Step 4: build virtual walls ----
        left_wall_x, right_wall_x, case = (
            self.build_virtual_walls(left_clusters, right_clusters)
        )

        if case == "NONE":
            self.get_logger().warn("NO LEGS DETECTED")
            self.pub_result.publish(self.bridge.cv2_to_imgmsg(canvas, 'bgr8'))
            return

        # ---- Step 5: compute corridor midpoint ----
        if case == "LR":
            mid_x = (left_wall_x + right_wall_x) / 2.0

        elif case == "L_only":
            mid_x = left_wall_x + self.half_lane_width_pixel

        else:  # R_only
            mid_x = right_wall_x - self.half_lane_width_pixel

        # ---- Step 6: moving-average filter (same as original) ----
        mid_x, self._mid_x_list = self.moving_average(
            mid_x, self._mid_x_list, self.filter_len
        )

        # ---- Step 7: draw result image ----
        result = cluster_canvas.copy()

        # Draw virtual walls
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

        # Draw midpoint line (red, same as original)
        cv2.line(result,
                 (int(mid_x), 0),
                 (int(mid_x), self.frame_height),
                 (0, 0, 255), 2)

        # ---- Step 8: compute offset and steering status ----
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

        # ---- Step 9: publish result ----
        self.pub_result.publish(
            self.bridge.cv2_to_imgmsg(result, 'bgr8')
        )


def main(args=None):
    rclpy.init(args=args)
    node = CorridorCenterDBScan()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()