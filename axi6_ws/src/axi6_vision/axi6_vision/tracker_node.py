#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
 
class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker_node")
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    node.get_logger().info("Tracker node is running...")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt (SIGINT) received. Shutting down...")
    finally:
        # This ensuring the node cleans up its network connections
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()