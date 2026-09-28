#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path


class LocalRansacPathNode(Node):
    def __init__(self):
        super().__init__('local_ransac_path_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('min_range', 0.30)
        self.declare_parameter('max_range', 5.00)
        self.declare_parameter('cluster_distance', 0.25)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('max_cluster_points', 60)
        self.declare_parameter('ransac_iterations', 120)
        self.declare_parameter('ransac_threshold', 0.25)
        self.declare_parameter('min_line_inliers', 3)
        self.declare_parameter('path_length', 4.0)
        self.declare_parameter('path_step', 0.5)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.cluster_distance = float(self.get_parameter('cluster_distance').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.max_cluster_points = int(self.get_parameter('max_cluster_points').value)
        self.ransac_iterations = int(self.get_parameter('ransac_iterations').value)
        self.ransac_threshold = float(self.get_parameter('ransac_threshold').value)
        self.min_line_inliers = int(self.get_parameter('min_line_inliers').value)
        self.path_length = float(self.get_parameter('path_length').value)
        self.path_step = float(self.get_parameter('path_step').value)

        self.marker_pub = self.create_publisher(MarkerArray, '/ransac_markers', 10)
        self.path_pub = self.create_publisher(Path, '/local_ransac_path', 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.get_logger().info(
            f'Local RANSAC Path Node started. Subscribing to {self.scan_topic}'
        )

    def scan_callback(self, scan_msg: LaserScan):
        points = self.scan_to_points(scan_msg)

        if len(points) < 10:
            self.publish_markers(scan_msg.header.frame_id, [], None, None, None)
            return

        clusters = self.cluster_points(points)
        centers = self.get_cluster_centers(clusters)

        if len(centers) < 4:
            self.publish_markers(scan_msg.header.frame_id, centers, None, None, None)
            return

        left_points, right_points = self.split_left_right(centers)

        left_line = self.fit_ransac_line(left_points)
        right_line = self.fit_ransac_line(right_points)

        center_line = None
        path_msg = None

        if left_line is not None and right_line is not None:
            center_line = self.compute_center_line(left_line, right_line)
            path_msg = self.create_path(scan_msg.header.frame_id, center_line)

        self.publish_markers(
            scan_msg.header.frame_id,
            centers,
            left_line,
            right_line,
            center_line
        )

        if path_msg is not None:
            self.path_pub.publish(path_msg)

    def scan_to_points(self, scan_msg: LaserScan):
        points = []
        angle = scan_msg.angle_min

        for r in scan_msg.ranges:
            if not math.isfinite(r):
                angle += scan_msg.angle_increment
                continue

            if r < self.min_range or r > self.max_range:
                angle += scan_msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)

            points.append((x, y))
            angle += scan_msg.angle_increment

        return points

    def cluster_points(self, points):
        if not points:
            return []

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            prev = points[i - 1]
            curr = points[i]

            dist = math.hypot(curr[0] - prev[0], curr[1] - prev[1])

            if dist <= self.cluster_distance:
                current_cluster.append(curr)
            else:
                if self.min_cluster_points <= len(current_cluster) <= self.max_cluster_points:
                    clusters.append(current_cluster)
                current_cluster = [curr]

        if self.min_cluster_points <= len(current_cluster) <= self.max_cluster_points:
            clusters.append(current_cluster)

        return clusters

    def get_cluster_centers(self, clusters):
        centers = []

        for cluster in clusters:
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]

            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)

            centers.append((cx, cy))

        return centers

    def split_left_right(self, centers):
        left = []
        right = []

        for x, y in centers:
            if y > 0.15:
                left.append((x, y))
            elif y < -0.15:
                right.append((x, y))

        return left, right

    def fit_ransac_line(self, points):
        if len(points) < self.min_line_inliers:
            return None

        best_line = None
        best_inliers = []

        for _ in range(self.ransac_iterations):
            p1, p2 = random.sample(points, 2)

            if abs(p2[0] - p1[0]) < 1e-6:
                continue

            m = (p2[1] - p1[1]) / (p2[0] - p1[0])
            b = p1[1] - m * p1[0]

            inliers = []

            for p in points:
                distance = abs(m * p[0] - p[1] + b) / math.sqrt(m * m + 1.0)

                if distance < self.ransac_threshold:
                    inliers.append(p)

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_line = (m, b)

        if best_line is None or len(best_inliers) < self.min_line_inliers:
            return None

        return best_line

    def compute_center_line(self, left_line, right_line):
        m_left, b_left = left_line
        m_right, b_right = right_line

        m_center = (m_left + m_right) / 2.0
        b_center = (b_left + b_right) / 2.0

        return (m_center, b_center)

    def create_path(self, frame_id, center_line):
        m, b = center_line

        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        x = 0.0
        while x <= self.path_length:
            y = m * x + b

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

            x += self.path_step

        return path

    def publish_markers(self, frame_id, centers, left_line, right_line, center_line):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 0

        for cx, cy in centers:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'blob_centers'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)
            marker_id += 1

        if left_line is not None:
            marker_array.markers.append(
                self.create_line_marker(frame_id, marker_id, 'left_ransac_line', left_line, 0.0, 1.0, 0.0)
            )
            marker_id += 1

        if right_line is not None:
            marker_array.markers.append(
                self.create_line_marker(frame_id, marker_id, 'right_ransac_line', right_line, 0.0, 0.0, 1.0)
            )
            marker_id += 1

        if center_line is not None:
            marker_array.markers.append(
                self.create_line_marker(frame_id, marker_id, 'center_path_line', center_line, 1.0, 0.0, 0.0)
            )

        self.marker_pub.publish(marker_array)

    def create_line_marker(self, frame_id, marker_id, namespace, line, r, g, b):
        m, c = line

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.06
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0

        for x in [0.0, self.path_length]:
            y = m * x + c
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.08
            marker.points.append(p)

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = LocalRansacPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
