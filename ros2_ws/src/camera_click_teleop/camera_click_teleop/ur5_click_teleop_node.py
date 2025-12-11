#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np


class Ur5ClickTeleopNode(Node):
    """
    Click interface for UR5:

    - subscribes to camera images
    - shows Camera View window with a horizontal center line
    - click ABOVE center  -> rotate base joint +delta
    - click BELOW center  -> rotate base joint -delta
    - publishes JointTrajectory to /scaled_joint_trajectory_controller/joint_trajectory
    """

    def __init__(self):
        super().__init__('ur5_click_teleop')

        # Parameters
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter(
            'trajectory_topic',
            '/scaled_joint_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter('base_joint_name', 'shoulder_pan_joint')
        self.declare_parameter('delta_angle', 0.1)      # radians per click
        self.declare_parameter('time_to_reach', 1.0)    # seconds

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        traj_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.base_joint_name = self.get_parameter('base_joint_name').get_parameter_value().string_value
        self.delta_angle = self.get_parameter('delta_angle').get_parameter_value().double_value
        self.time_to_reach = self.get_parameter('time_to_reach').get_parameter_value().double_value

        self.get_logger().info("UR5 Click Teleop starting.")
        self.get_logger().info(f"Subscribing to image topic: {image_topic}")
        self.get_logger().info(f"Publishing trajectories to: {traj_topic}")
        self.get_logger().info(f"Base joint: {self.base_joint_name}, delta: {self.delta_angle} rad")

        self.bridge = CvBridge()
        self.latest_frame = None

        # Simple internal target angle for base joint
        self.current_angle = 0.0

        # Publisher for trajectory
        self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)

        # Image subscriber
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # OpenCV window + timer
        self.window_name = "Camera View (UR5)"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)
        self.window_timer = self.create_timer(1.0 / 30.0, self.update_window)

    # ----------------- Callbacks ----------------- #

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def update_window(self):
        # Always show black background (ignore real camera image)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

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

        h, _ = self.latest_frame.shape[:2]
        center_y = h // 2

        if y < center_y:
            self.current_angle += self.delta_angle
            direction = "+delta"
        else:
            self.current_angle -= self.delta_angle
            direction = "-delta"

        self.get_logger().info(
            f"Click y={y}, center_y={center_y} -> {direction}, "
            f"target angle={self.current_angle:.3f} rad"
        )

        traj = JointTrajectory()
        traj.joint_names = [self.base_joint_name]

        point = JointTrajectoryPoint()
        point.positions = [self.current_angle]
        point.time_from_start.sec = int(self.time_to_reach)
        point.time_from_start.nanosec = int(
            (self.time_to_reach - int(self.time_to_reach)) * 1e9
        )

        traj.points.append(point)
        self.traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = Ur5ClickTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
