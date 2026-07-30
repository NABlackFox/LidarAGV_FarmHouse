#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

try:
    from cv_bridge import CvBridge
    import cv2
    _HAS_CV = True
except ImportError:
    _HAS_CV = False


class IntervalImageCaptureService(Node):
    """Saves images to disk at a 5s interval when enabled via SetBool service."""

    def __init__(self):
        super().__init__('interval_image_capture_service')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('image_topic', '/top_camera/image_raw')
        self.declare_parameter('save_dir', '/tmp/orlar_pictures')
        self.declare_parameter('image_format', 'png')
        self.declare_parameter('filename_prefix', 'interval_capture')
        self.declare_parameter('capture_interval', 5.0)  # 5 seconds interval

        self.image_topic = self.get_parameter('image_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.image_format = self.get_parameter('image_format').value.lower()
        self.filename_prefix = self.get_parameter('filename_prefix').value
        self.capture_interval = float(self.get_parameter('capture_interval').value)

        os.makedirs(self.save_dir, exist_ok=True)

        if not _HAS_CV:
            self.get_logger().error(
                "cv_bridge / OpenCV not found. Install with:\n"
                "  sudo apt install ros-$ROS_DISTRO-cv-bridge python3-opencv"
            )

        self.bridge = CvBridge() if _HAS_CV else None
        self._latest_frame = None
        self.enable_capture = False  # Default OFF

        # Subscriber
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, qos
        )

        # Service to toggle ON/OFF
        self._toggle_srv = self.create_service(
            SetBool, '/camera/enable_recording', self._toggle_capture_callback
        )

        # Background Timer for interval capture
        if self.capture_interval > 0:
            self._timer = self.create_timer(
                self.capture_interval, self._save_image_callback
            )

        self.get_logger().info(
            f"Interval Image Capture Service Ready.\n"
            f"  Topic    : {self.image_topic}\n"
            f"  Interval : Every {self.capture_interval}s\n"
            f"  Service  : /camera/enable_recording (std_srvs/SetBool)"
        )

    def _toggle_capture_callback(self, request, response):
        self.enable_capture = request.data
        status = "ENABLED" if self.enable_capture else "DISABLED"
        self.get_logger().info(f"Camera interval recording has been {status}")
        
        response.success = True
        response.message = f"Recording status successfully set to {self.enable_capture}"
        return response

    def _image_callback(self, msg: Image):
        if not _HAS_CV:
            return
        try:
            self._latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert frame: {e}")

    def _save_image_callback(self):
        if not self.enable_capture or not _HAS_CV or self._latest_frame is None:
            return

        millis = int((time.time() % 1) * 1000)
        timestamp = time.strftime('%Y%m%d_%H%M%S') + f"_{millis:03d}"
        filename = f"{self.filename_prefix}_{timestamp}.{self.image_format}"
        filepath = os.path.join(self.save_dir, filename)

        try:
            cv2.imwrite(filepath, self._latest_frame)
            self.get_logger().info(f"Interval frame saved: {filepath}")
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