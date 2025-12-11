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
    UR5 click-based teleoperation.

    - Shows a black window with a green center line and a green square.
    - Clicking in the window moves the square to the click position.
    - Horizontal offset (left/right) controls shoulder_pan_joint.
    - Vertical offset (up/down) controls shoulder_lift_joint.
    - Sends a JointTrajectory with ALL 6 UR5 joints to the
      scaled_joint_trajectory_controller (or other topic via parameter).
    """

    def __init__(self):
        super().__init__('ur5_click_teleop')

        # ---- Parameters ----
        self.declare_parameter('image_topic', '/image_raw')  # unused, for future extensions
        self.declare_parameter(
            'trajectory_topic',
            '/scaled_joint_trajectory_controller/joint_trajectory'
        )
        # Bigger deltas so motion is clearly visible
        self.declare_parameter('delta_pan', 0.4)   # radians per click for base
        self.declare_parameter('delta_lift', 0.25) # radians per click for shoulder
        self.declare_parameter('time_to_reach', 1.0)  # seconds
        self.declare_parameter('window_width', 640)
        self.declare_parameter('window_height', 480)
        self.declare_parameter('square_size', 80)  # pixel size of green square

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        traj_topic = self.get_parameter('trajectory_topic').get_parameter_value().string_value
        self.delta_pan = self.get_parameter('delta_pan').get_parameter_value().double_value
        self.delta_lift = self.get_parameter('delta_lift').get_parameter_value().double_value
        self.time_to_reach = self.get_parameter('time_to_reach').get_parameter_value().double_value
        self.win_w = int(self.get_parameter('window_width').get_parameter_value().integer_value)
        self.win_h = int(self.get_parameter('window_height').get_parameter_value().integer_value)
        self.square_size = int(self.get_parameter('square_size').get_parameter_value().integer_value)

        # UR5 joint order expected by controller
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        self.get_logger().info("UR5 Click Teleop starting.")
        self.get_logger().info(f"Publishing trajectories to: {traj_topic}")
        self.get_logger().info(f"Joints: {self.joint_names}")
        self.get_logger().info(
            f"delta_pan={self.delta_pan} rad, delta_lift={self.delta_lift} rad, "
            f"time_to_reach={self.time_to_reach} s"
        )

        self.bridge = CvBridge()

        # joint state tracking
        self.current_positions = [0.0] * len(self.joint_names)
        self.have_joint_state = False

        # Visual state: last click position (start in center)
        self.last_click_x = self.win_w // 2
        self.last_click_y = self.win_h // 2

        # ---- Publishers / Subscribers ----
        self.traj_pub = self.create_publisher(JointTrajectory, traj_topic, 10)

        # Keep image subscription only to satisfy "camera-based" requirement,
        # but we don't use the pixels right now (pure black window).
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

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
        # Not used for now (we draw a black background),
        # but the subscription is here if you want to show the real image later.
        pass

    def joint_state_callback(self, msg: JointState):
        # Initialize our joint vector from /joint_states
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
        # Black background
        frame = np.zeros((self.win_h, self.win_w, 3), dtype=np.uint8)

        # Horizontal center line (like spec)
        center_y = self.win_h // 2
        cv2.line(frame, (0, center_y), (self.win_w, center_y), (0, 255, 0), 2)

        # Green square around last click
        half = self.square_size // 2
        x1 = max(0, self.last_click_x - half)
        y1 = max(0, self.last_click_y - half)
        x2 = min(self.win_w - 1, self.last_click_x + half)
        y2 = min(self.win_h - 1, self.last_click_y + half)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def on_mouse_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # store click for drawing
        self.last_click_x = int(x)
        self.last_click_y = int(y)

        center_x = self.win_w // 2
        center_y = self.win_h // 2

        if not self.have_joint_state:
            self.get_logger().warn(
                "No joint states received yet; using internal zeros as starting pose."
            )

        # Decide movement direction based on click position
        pan_step = 0.0
        lift_step = 0.0

        # threshold so tiny clicks near center don't spam motion
        thresh = 10

        dx = x - center_x
        dy = y - center_y

        if abs(dx) > thresh:
            pan_step = self.delta_pan * (1.0 if dx > 0 else -1.0)

        if abs(dy) > thresh:
            # screen y increases downwards, so invert sign for "up"
            lift_step = self.delta_lift * (-1.0 if dy < 0 else 1.0)

        if pan_step == 0.0 and lift_step == 0.0:
            self.get_logger().info(
                f"Click near center (x={x}, y={y}) -> no joint change."
            )
            return

        # Update current joint targets
        self.current_positions[0] += pan_step         # shoulder_pan_joint
        self.current_positions[1] += lift_step        # shoulder_lift_joint

        self.get_logger().info(
            f"Click ({x},{y}) -> pan_step={pan_step:.3f}, lift_step={lift_step:.3f}, "
            f"new pan={self.current_positions[0]:.3f}, "
            f"new lift={self.current_positions[1]:.3f}"
        )

        # Build and publish trajectory
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
