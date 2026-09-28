import math
import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class A2M12StructureFilter(Node):

    def __init__(self):
        super().__init__('a2m12_structure_filter')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_hector_clean')

        self.declare_parameter('min_range', 0.30)
        self.declare_parameter('near_range', 4.00)
        self.declare_parameter('max_range', 8.00)

        self.declare_parameter('near_min_width', 0.015)
        self.declare_parameter('far_min_width', 0.100)

        self.input_topic = self.get_parameter(
            'input_topic').value
        self.output_topic = self.get_parameter(
            'output_topic').value

        self.min_range = self.get_parameter(
            'min_range').value
        self.near_range = self.get_parameter(
            'near_range').value
        self.max_range = self.get_parameter(
            'max_range').value

        self.near_min_width = self.get_parameter(
            'near_min_width').value
        self.far_min_width = self.get_parameter(
            'far_min_width').value

        self.pub = self.create_publisher(
            LaserScan, self.output_topic, 10)

        self.sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            10
        )

        self.get_logger().info(
            'A2M12 structure filter ready: '
            '0.30-4.00 m >=15 mm, '
            '4.00-8.00 m >=100 mm'
        )

    def point_xy(self, index, r, msg):
        angle = msg.angle_min + index * msg.angle_increment
        return (
            r * math.cos(angle),
            r * math.sin(angle)
        )

    def point_gap(self, i1, i2, ranges, msg):
        x1, y1 = self.point_xy(i1, ranges[i1], msg)
        x2, y2 = self.point_xy(i2, ranges[i2], msg)
        return math.hypot(x2 - x1, y2 - y1)

    def cluster_width(self, cluster, ranges, msg):
        if not cluster:
            return 0.0

        mean_range = sum(ranges[i] for i in cluster) / len(cluster)

        if len(cluster) == 1:
            return mean_range * abs(msg.angle_increment)

        width = 0.0

        for a, b in zip(cluster[:-1], cluster[1:]):
            width += self.point_gap(a, b, ranges, msg)

        # Compensate approximately one angular cell.
        width += mean_range * abs(msg.angle_increment)

        return width
    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        n = len(ranges)

        output = copy.deepcopy(msg)
        output.ranges = [float('inf')] * n

        valid = [
            i for i, r in enumerate(ranges)
            if math.isfinite(r)
            and self.min_range <= r <= self.max_range
        ]

        if not valid:
            self.pub.publish(output)
            return

        cluster_gap = 0.12

        clusters = []
        current = [valid[0]]

        for i in valid[1:]:
            previous = current[-1]

            consecutive = (i == previous + 1)

            if consecutive:
                gap = self.point_gap(
                    previous, i, ranges, msg
                )
            else:
                gap = float('inf')

            if consecutive and gap <= cluster_gap:
                current.append(i)
            else:
                clusters.append(current)
                current = [i]

        clusters.append(current)

        kept_clusters = 0
        removed_clusters = 0
        kept_rays = 0
        for cluster in clusters:
            mean_range = (
                sum(ranges[i] for i in cluster)
                / len(cluster)
            )

            width = self.cluster_width(
                cluster, ranges, msg
            )

            if mean_range <= self.near_range:
                # 15 mm target with 3 mm measurement margin.
                threshold = self.near_min_width - 0.003
            else:
                threshold = self.far_min_width

            if width >= threshold:
                kept_clusters += 1

                for i in cluster:
                    output.ranges[i] = ranges[i]
                    kept_rays += 1
            else:
                removed_clusters += 1

        self.pub.publish(output)

        self._scan_count = getattr(
            self, '_scan_count', 0
        ) + 1

        if self._scan_count % 50 == 0:
            self.get_logger().info(
                f'clusters kept={kept_clusters}, '
                f'removed={removed_clusters}, '
                f'rays kept={kept_rays}/{len(valid)}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = A2M12StructureFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
