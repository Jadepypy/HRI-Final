#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        # Subscribe to depth topic (remapped in launch file)
        self.sub = self.create_subscription(
            Image,
            '/oakd/stereo/depth',
            self.depth_callback,
            qos
        )
        # Publisher for obstacle events
        self.pub = self.create_publisher(String, '/obstacle_event', 10)
        self.get_logger().info("ObstacleDetector node started")

    def depth_callback(self, msg: Image):
        # For now, just publish a test event whenever a frame arrives
        event = String()
        event.data = "turn_left"   # Replace with real logic later
        self.pub.publish(event)
        self.get_logger().info("Published obstacle event")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
