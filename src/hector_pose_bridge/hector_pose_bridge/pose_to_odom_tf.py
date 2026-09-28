import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class HectorPoseBridge(Node):

    def __init__(self):
        super().__init__('hector_pose_bridge')

        self.declare_parameter(
            'input_topic',
            '/hector_pose'
        )
        self.declare_parameter(
            'odom_frame',
            'odom'
        )
        self.declare_parameter(
            'base_frame',
            'base_link'
        )

        self.input_topic = self.get_parameter(
            'input_topic'
        ).value

        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.pose_callback,
            20
        )

        self.get_logger().info(
            f'Hector pose bridge: '
            f'{self.input_topic} -> '
            f'{self.odom_frame} -> {self.base_frame}'
        )

    def pose_callback(self, msg):
        tf_msg = TransformStamped()

        # Preserve Hector/LiDAR timestamp.
        tf_msg.header.stamp = msg.header.stamp

        # Deliberately reinterpret Hector local coordinates
        # as the odometry coordinate frame.
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        tf_msg.transform.translation.x = msg.pose.position.x
        tf_msg.transform.translation.y = msg.pose.position.y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)

    node = HectorPoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
