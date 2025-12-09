#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class ClickTeleopNode(Node):
    def __init__(self):
        super().__init__('click_teleop_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('forward_speed', 0.1)
        self.declare_parameter('backward_speed', -0.1)
        self.declare_parameter('command_duration', 0.5)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.backward_speed = self.get_parameter('backward_speed').get_parameter_value().double_value
        self.command_duration = self.get_parameter('command_duration').get_parameter_value().double_value

        self.get_logger().info("ClickTeleopNode starting.")
        self.get_logger().info(f"Subscribing to image topic: {image_topic}")
        self.get_logger().info(f"Publishing cmd_vel to: {cmd_vel_topic}")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.stop_timer = None

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # OpenCV window
        self.window_name = "Camera View"
        cv2.namedWindow(self.window_name)

        # Timer to refresh the window ~30 FPS
        self.window_timer = self.create_timer(1.0 / 30.0, self.update_window)

        # Mouse callback
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)

    # --------------------- Callbacks --------------------- #

    def image_callback(self, msg: Image):
        """Store latest frame; window is updated by timer."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame
            self.get_logger().debug("Image callback called.")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def update_window(self):
        """Periodically refresh the OpenCV window."""
        if self.latest_frame is None:
            # Show a black image until camera provides frames
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            frame = self.latest_frame.copy()

        # Draw horizontal center line
        h, w = frame.shape[:2]
        center_y = h // 2
        cv2.line(frame, (0, center_y), (w, center_y), (0, 255, 0), 2)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.latest_frame is None:
            self.get_logger().warn("Clicked but no frame yet.")
            return

        h, w = self.latest_frame.shape[:2]
        center_y = h // 2

        twist = Twist()
        if y < center_y:
            twist.linear.x = self.forward_speed
            direction = "FORWARD"
        else:
            twist.linear.x = self.backward_speed
            direction = "BACKWARD"

        self.get_logger().info(
            f"Mouse click at y={y}, center_y={center_y} → {direction}"
        )
        self.cmd_pub.publish(twist)

        # one-shot movement then stop
        if self.stop_timer is not None:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(self.command_duration, self._send_stop_once)

    def _send_stop_once(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = ClickTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down ClickTeleopNode.")
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
