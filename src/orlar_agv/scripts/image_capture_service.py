#!/usr/bin/env python3
"""
image_capture_service.py
=========================

Implements the "take a picture" service mechanism for the top-mounted
table-inspection camera (see top_camera.xacro / camera_frame.xacro).

Why a service instead of just using the raw image topic
---------------------------------------------------------
The simulated/real camera continuously PUBLISHES a video stream on
    /top_camera/image_raw          (sensor_msgs/Image)
That is fine for live viewing, but for inspection tasks you usually want
a single still photo taken at a specific moment (e.g. "robot has arrived
at the table, NOW take the picture"). Streaming video is the wrong tool
for that — you want a synchronous request/response, which is exactly
what a ROS 2 *service* provides.

This node:
  1. Subscribes to /top_camera/image_raw and keeps only the latest frame
     in memory (it does NOT save every frame — that would just be
     recording video again).
  2. Advertises a service  /top_camera/take_picture  of type
     std_srvs/srv/Trigger.
  3. When the service is called:
        - grabs the most recent frame currently held in memory
        - saves it to disk as a PNG/JPEG file (using OpenCV)
        - returns success=True and the saved file path in the response
          message, OR success=False with an error message if no frame
          has been received yet.

Service signature (std_srvs/srv/Trigger)
------------------------------------------
    Request:  (empty)
    Response:
        bool    success
        string  message      # contains the saved file path on success

Usage
-----
    ros2 run orlar_agv image_capture_service.py
        --ros-args -p save_dir:=/tmp/orlar_pictures -p image_format:=png

    # In another terminal, trigger a capture:
    ros2 service call /top_camera/take_picture std_srvs/srv/Trigger {}
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

try:
    from cv_bridge import CvBridge
    import cv2
    _HAS_CV = True
except ImportError:
    # Fallback path if cv_bridge / OpenCV are not installed: we still
    # let the node run and subscribe, but image saving will raise a
    # clear error rather than crashing at import time.
    _HAS_CV = False


class ImageCaptureService(Node):
    """Holds the latest camera frame and saves it to disk on request."""

    def __init__(self):
        super().__init__('image_capture_service')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('image_topic', '/top_camera/image_raw')
        self.declare_parameter('save_dir', '/tmp/orlar_pictures')
        self.declare_parameter('image_format', 'png')   # 'png' or 'jpg'
        self.declare_parameter('filename_prefix', 'table_capture')

        self.image_topic = self.get_parameter('image_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.image_format = self.get_parameter('image_format').value.lower()
        self.filename_prefix = self.get_parameter('filename_prefix').value

        if self.image_format not in ('png', 'jpg', 'jpeg'):
            self.get_logger().warn(
                f"Unsupported image_format '{self.image_format}', "
                f"defaulting to 'png'."
            )
            self.image_format = 'png'

        os.makedirs(self.save_dir, exist_ok=True)

        if not _HAS_CV:
            self.get_logger().error(
                "cv_bridge / OpenCV not found. Install with:\n"
                "  sudo apt install ros-$ROS_DISTRO-cv-bridge python3-opencv\n"
                "The service will still start, but capture requests will fail."
            )

        self.bridge = CvBridge() if _HAS_CV else None
        self._latest_frame = None        # last cv2 image (BGR) received
        self._latest_stamp = None        # ROS time of that frame

        # ---- Subscriber: keep only the most recent frame -----------------
        # BEST_EFFORT + KEEP_LAST(1) -> we don't care about buffering a
        # video history, only "what does the camera see right now".
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, qos
        )

        # ---- Service: take a single picture on demand --------------------
        self._srv = self.create_service(
            Trigger, '/top_camera/take_picture', self._take_picture_callback
        )

        self.get_logger().info(
            f"Image capture service ready.\n"
            f"  Subscribed to : {self.image_topic}\n"
            f"  Service       : /top_camera/take_picture (std_srvs/Trigger)\n"
            f"  Save directory: {self.save_dir}"
        )

    # -----------------------------------------------------------------
    def _image_callback(self, msg: Image):
        """Cache the latest frame. This does NOT write anything to disk —
        saving only happens when the service is actually called."""
        if not _HAS_CV:
            return
        try:
            self._latest_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
            self._latest_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f"Failed to convert incoming image: {e}")

    # -----------------------------------------------------------------
    def _take_picture_callback(self, request, response):
        """Service handler: save the most recent cached frame to disk."""
        if not _HAS_CV:
            response.success = False
            response.message = "cv_bridge/OpenCV not installed on this system."
            return response

        if self._latest_frame is None:
            response.success = False
            response.message = (
                f"No image received yet on topic '{self.image_topic}'. "
                "Is the camera sensor running?"
            )
            self.get_logger().warn(response.message)
            return response

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = f"{self.filename_prefix}_{timestamp}.{self.image_format}"
        filepath = os.path.join(self.save_dir, filename)

        try:
            cv2.imwrite(filepath, self._latest_frame)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save image: {e}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = filepath
        self.get_logger().info(f"Picture captured and saved to: {filepath}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
