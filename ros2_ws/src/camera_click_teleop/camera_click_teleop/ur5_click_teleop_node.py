#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np


class Ur5ClickTeleopNode(Node):
    """
    Click interface for UR5:

    - Shows a black window with a green center line
    - Click ABOVE center  -> increase base joint angle
    - Click BELOW center  -> decrease base joint angle
    - Publishes JointTrajectory with ALL 6 UR5 joints to the
      scaled_joint_trajectory_controller (or whatever topic you configure)
    """

    def __init__(self):
        super().__init__('ur5_click_teleop')

        # ---- Parameters ----
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter(
            'trajectory_topic',
            '/scaled_joint_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter('delta_angle', 0.1)   # radians per click
        self.declare_parameter('time_to_reach', 1.0) # seconds

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        traj_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.delta_angle = self.get_parameter('delta_angle').get_parameter_value().double_value
        self.time_to_reach = self.get_parameter('time_to_reach').get_parameter_value().double_value

        # UR5 joint order expected by the controller
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.get_logger().info("UR5 Click Teleop starting.")
        self.get_logger().info(f"Subscribing to image topic: {image_topic}")
        self.get_logger().info(f"Publishing trajectories to: {traj_topic}")
        self.get_logger().info(f"Joints: {self.joint_names}")
        self.get_logger().info(f"delta_angle={self.delta_angle} rad, time_to_reach={self.time_to_reach} s")

        self.bridge = CvBridge()

        # current desired joint positions (we'll fill from joint states)
        self.current_positions = [0.0] * len(self.joint_names)
        self.have_joint_state = False

        # Publisher for trajectory
        self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)

        # Image subscriber (not used visually, but kept for requirement)
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Joint state subscriber to initialize joint angles from real robot
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # OpenCV window + timer
        self.window_name = "Camera View (UR5)"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_mouse_click)
        self.window_timer = self.create_timer(1.0 / 30.0, self.update_window)

    # ----------------- Callbacks ----------------- #

    def image_callback(self, msg: Image):
        # We ignore actual image contents (black window only),
        # but keep this in case you want to use it later.
        pass

    def joint_state_callback(self, msg: JointState):
        # Map incoming joint state to our joint order
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        updated = False
        for i, jn in enumerate(self.joint_names):
            if jn in name_to_pos:
                self.current_positions[i] = name_to_pos[jn]
                updated = True
        if updated and not self.have_joint_state:
            self.have_joint_state = True
            self.get_logger().info(
                f"Initialized joint positions from /joint_states: {self.current_positions}"
            )

    def update_window(self):
        # Always show black background with green center line
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        h, w = frame.shape[:2]
        center_y = h // 2
        cv2.line(frame, (0, center_y), (w, center_y), (0, 255, 0), 2)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # Use fixed image height (same as update_window)
        h = 480
        center_y = h // 2

        if not self.have_joint_state:
            self.get_logger().warn(
                "No joint states received yet; using internal zeros as starting pose."
            )

        if y < center_y:
            delta = self.delta_angle
            direction = "+delta"
        else:
            delta = -self.delta_angle
            direction = "-delta"

        # base joint is index 0 (shoulder_pan_joint)
        self.current_positions[0] += delta

        self.get_logger().info(
            f"Click y={y}, center_y={center_y} -> {direction}, "
            f"new base angle={self.current_positions[0]:.3f} rad"
        )

        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(self.current_positions)
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
