# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------


import cv2
import time
import rclpy
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ArducamNode(Node):
    """RGB publisher for an Arducam UVC USB camera (e.g. B0CLXZ29F9).

    Unlike the RealSense/ZED, this is a plain UVC webcam: no depth, no vendor
    SDK -- it is driven through OpenCV's V4L2 backend. Address it by a stable
    /dev/v4l/by-id/... path (preferred, survives replug/reboot) or a numeric
    /dev/video index.
    """

    def __init__(self):
        super().__init__('arducam_node')

        # `device` may be a numeric index ("0" -> /dev/video0) or a path such as
        # /dev/v4l/by-id/usb-Arducam_...-video-index0 (stable across reboots).
        self.declare_parameter('device', "0")
        self.declare_parameter('name', "arducam")
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)

        self.device = self.get_parameter('device').value
        self.name = self.get_parameter('name').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.camera_fps = self.get_parameter('fps').value

        self.initialize_camera()

        self.image_pub = self.create_publisher(Image, f'/arducam/{self.name}/im', 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(1 / self.camera_fps, self.timer_callback)

    def initialize_camera(self):
        # A bare digit string is a /dev/video index; anything else is a path.
        source = int(self.device) if str(self.device).isdigit() else self.device
        self.cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open Arducam at '{self.device}'")
            exit()

        # MJPG is required to hit 1080p30 over USB2 -- raw YUYV maxes out far lower.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
        # Keep latency low: don't let frames pile up in the driver buffer.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"Opened Arducam '{self.device}' at {actual_w}x{actual_h}"
        )
        if (actual_w, actual_h) != (self.width, self.height):
            self.get_logger().warn(
                f"Requested {self.width}x{self.height} but got {actual_w}x{actual_h}"
            )

    def timer_callback(self):
        start = time.time()
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("Dropped frame from Arducam")
            return

        # OpenCV delivers BGR; publish rgb8 to match the other camera nodes.
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image_msg = self.bridge.cv2_to_imgmsg(image_rgb, encoding="rgb8")
        self.image_pub.publish(image_msg)

        effective_hertz = 1 / (time.time() - start)
        if effective_hertz < self.camera_fps - 10:
            self.get_logger().warn(f"WARNING: Effective hz: {effective_hertz}")

    def destroy_node(self):
        self.cap.release()
        self.get_logger().info("Destroyed Arducam Node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    arducam_node = ArducamNode()
    try:
        rclpy.spin(arducam_node)
    except KeyboardInterrupt:
        pass
    finally:
        arducam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
