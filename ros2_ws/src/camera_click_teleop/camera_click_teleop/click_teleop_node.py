#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2


class ClickTeleopNode(Node):
    def __init__(self):
        super().__init__('click_teleop_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('forward_speed', 0.1)
        self.declare_parameter('backward_speed', -0.1)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.backward_speed = self.get_parameter('backward_speed').get_parameter_value().double_value

        self.get_logger().info("ClickTeleopNode starting.")
        self.get_logger().info(f"Subscribing to image topic: {image_topic}")
        self.get_logger().info(f"Publishing cmd_vel to: {cmd_vel_topic}")

        self.bridge = CvBridge()
        self.latest_frame = None

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.window_name = "Camera View"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)

    def on_mouse_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.latest_frame is None:
            self.get_logger().warn("Mouse click but no frame yet.")
            return

        height, width = self.latest_frame.shape[:2]
        center_y = height // 2

        twist = Twist()

        if y < center_y:
            twist.linear.x = self.forward_speed
            self.get_logger().info(f"Clicked ABOVE center (y={y} < {center_y}) → FORWARD")
        else:
            twist.linear.x = self.backward_speed
            self.get_logger().info(f"Clicked BELOW center (y={y} >= {center_y}) → BACKWARD")

        self.cmd_pub.publish(twist)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")


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
