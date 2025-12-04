#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ClickTeleopNode(Node):
    def __init__(self):
        super().__init__('click_teleop_node')
        self.get_logger().info("ClickTeleopNode initialized (no logic yet).")

def main(args=None):
    rclpy.init(args=args)
    node = ClickTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
