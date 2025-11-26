import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()

        self.depth_topic = '/stereo/converted_depth'
        self.color_topic = '/oakd/rgb/preview/image_raw'

        self.thresh_m = 2.0
        self.crop_ratio = 0.3
        self.yellow_percent_threshold = 3

        self.last_state = 'clear'

        self.obstacle_frames_needed = 1
        self.clear_frames_needed = 2
        self.yellow_frames_needed = 2

        self.obstacle_count = 0
        self.clear_count = 0
        self.yellow_count = 0

        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.create_subscription(Image, self.color_topic, self.on_color, 10)

        self.pub = self.create_publisher(String, '/obstacle_event', 10)

        self.get_logger().info(
            f"ObstacleDetector running — depth:{self.depth_topic}  color:{self.color_topic}"
        )

    def on_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")
            return

        depth_m = depth.astype(np.float32) / 1000.0
        self.process_depth(depth_m)

    def process_depth(self, depth):
        h, w = depth.shape
        ch = int(h * self.crop_ratio)
        cw = int(w * self.crop_ratio)
        y0 = (h - ch) // 2
        x0 = (w - cw) // 2
        roi = depth[y0:y0+ch, x0:x0+cw]

        valid = roi[roi > 0.05]
        if valid.size == 0:
            return

        median_m = float(np.median(valid))

        if median_m < self.thresh_m:
            self.obstacle_count += 1
            self.clear_count = 0
        else:
            self.clear_count += 1
            self.obstacle_count = 0

        if self.obstacle_count >= self.obstacle_frames_needed and self.last_state != 'turn_left':
            self.last_state = 'turn_left'
            self.pub.publish(String(data='turn_left'))
            self.get_logger().info(f"Obstacle detected (median={median_m:.2f} m) -> turn_left")

        elif self.clear_count >= self.clear_frames_needed and self.last_state == 'turn_left':
            self.last_state = 'clear'
            self.pub.publish(String(data='clear'))
            self.get_logger().info("Path clear -> resume")

    def on_color(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([10, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        yellow_percent = (np.count_nonzero(mask) / mask.size) * 100.0
        self.get_logger().info(f"Yellow percent: {yellow_percent:.2f}")

        if yellow_percent > self.yellow_percent_threshold:
            self.yellow_count += 1
        else:
            self.yellow_count = 0

        if self.yellow_count >= self.yellow_frames_needed and self.last_state != 'STOP_YELLOW':
            self.last_state = 'STOP_YELLOW'
            self.pub.publish(String(data='STOP_YELLOW'))
            self.get_logger().info("Yellow detected -> stop")

        elif yellow_percent < self.yellow_percent_threshold and self.last_state == 'STOP_YELLOW':
            self.last_state = 'clear'
            self.pub.publish(String(data='clear'))
            self.get_logger().info("Yellow cleared -> resume")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
