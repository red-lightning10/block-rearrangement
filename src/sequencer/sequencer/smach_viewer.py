"""
SMACH Viewer - Launch SMACH viewer
"""

import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    
    node = Node('smach_viewer')
    node.get_logger().info('Launch SMACH viewer with: ros2 run smach_viewer smach_viewer.py')
    node.get_logger().info('Then connect to: /SM_PICK')
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

