#!/usr/bin/env python3
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage
from custom_interfaces.srv import SetCameraState

from cv_bridge import CvBridge
import cv2


class IntervalImageCaptureService(Node):
    """Saves images to disk using table ID and timestamp when enabled."""

    def __init__(self):
        super().__init__('interval_image_capture_service')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('image_topic', '/top_camera/image_raw/compressed')
        self.declare_parameter('save_dir', '/tmp/orlar_pictures')
        self.declare_parameter('image_format', 'png')
        self.declare_parameter('capture_interval', 5.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.image_format = self.get_parameter('image_format').value.lower()
        self.capture_interval = float(self.get_parameter('capture_interval').value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self._latest_frame = None
        self.enable_capture = False
        self.current_table_id = "UNKNOWN"

        # subscriber -- CompressedImage, best-effort to match the driver
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            CompressedImage, self.image_topic, self._image_callback, qos
        )

        # service -- lets route_follow (or anyone) toggle capture + tag table_id
        self._toggle_srv = self.create_service(
            SetCameraState, '/camera/set_state', self._toggle_capture_callback
        )

        # timer -- fires independently of the image callback
        if self.capture_interval > 0:
            self._timer = self.create_timer(
                self.capture_interval, self._save_image_callback
            )

        self.get_logger().info(
            f"Interval Image Capture Service Ready.\n"
            f"  Topic    : {self.image_topic}\n"
            f"  Save dir : {self.save_dir}\n"
            f"  Interval : Every {self.capture_interval}s\n"
            f"  Service  : /camera/set_state"
        )

    # --- service ------------------------------------------------------------
    def _toggle_capture_callback(self, request, response):
        self.enable_capture = request.enable
        self.current_table_id = request.table_id.strip() if request.table_id else "UNKNOWN"

        status = "ENABLED" if self.enable_capture else "DISABLED"
        self.get_logger().info(
            f"Camera recording {status} for Table ID: '{self.current_table_id}'"
        )

        response.success = True
        response.message = f"Recording status set to {self.enable_capture} for {self.current_table_id}"
        return response

    # --- image callback -------------------------------------------------
    def _image_callback(self, msg: CompressedImage):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert frame: {e}")
            return

        # validity check: cv2.imdecode (inside cv_bridge) can return None
        # or a zero-size array on a truncated/corrupt compressed frame
        # WITHOUT raising -- catching that here is what stops garbage
        # frames from ever reaching self._latest_frame.
        if frame is None or frame.size == 0:
            self.get_logger().warn(
                f"Decoded frame invalid (msg.data={len(msg.data)} bytes), skipping."
            )
            return

        self._latest_frame = frame

    # --- capture timer --------------------------------------------------
    def _save_image_callback(self):
        if not self.enable_capture or self._latest_frame is None:
            return

        # safe timestamp without colons: YYYY-MM-DD_HH-MM-SS
        now = datetime.now()
        timestamp = now.strftime('%Y-%m-%d_%H-%M-%S')

        filename = f"{self.current_table_id}_{timestamp}.{self.image_format}"
        filepath = os.path.join(self.save_dir, filename)

        try:
            success = cv2.imwrite(filepath, self._latest_frame)
            if success:
                self.get_logger().info(f"Interval frame saved: {filepath}")
            else:
                self.get_logger().error(f"cv2.imwrite returned False for path: {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save frame: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = IntervalImageCaptureService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()