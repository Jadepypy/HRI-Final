import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
import time


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()

        self.STEP_SIZE = 0.5
        self.sensor_state = {
            "obstacle_right": False,
            "obstacle_front": False,
            "obstacle_left": False
        }
        self.latest_depth = None

        # time-based logging
        self.last_log_time = 0.0
        self.LOG_INTERVAL = 10.0   # seconds between log messages

        self.depth_topic = '/stereo/converted_depth'
        # self.color_topic = '/oakd/rgb/preview/image_raw'

        self.thresh_m = 2.0
        self.crop_ratio = 0.3
        # self.yellow_percent_threshold = 3

        self.last_state = 'clear'

        self.obstacle_frames_needed = 1
        self.clear_frames_needed = 2
        self.yellow_frames_needed = 2

        self.obstacle_count = 0
        self.clear_count = 0
        self.yellow_count = 0

        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        # self.create_subscription(Image, self.color_topic, self.on_color, 10)

        self.pub = self.create_publisher(String, '/obstacle_event', 10)

        self.get_logger().info(
            f"ObstacleDetector running — depth:{self.depth_topic}"
        )

    def on_depth(self, msg):
        """Standard Perception Logic from before"""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            depth_m = depth.astype(np.float32) / 1000.0
            h, w = depth_m.shape
            crop_w = int(w * 0.2)
            x_start = (w - crop_w) // 2

            #valid_front = front_view[front_view > 0.05]
            # Slice Regions
            # Front (Center)
            #front_view = depth_m[:, col_w:2 * col_w]
           # valid_front = front_view[front_view > 0.05]
            front_view = depth_m[:, x_start : x_start + crop_w]
            valid_front = front_view[front_view > 0.05]
            dist_front = np.median(valid_front) if valid_front.size > 0 else 9.9

            # Right
           # right_view = depth_m[:, 2 * col_w:]
           # valid_right = right_view[right_view > 0.05]
           # dist_right = np.median(valid_right) if valid_right.size > 0 else 9.9

            # Left
           # left_view = depth_m[:, :col_w]
           # valid_left = left_view[left_view > 0.05]
           # dist_left = np.median(valid_left) if valid_left.size > 0 else 9.9

            # Update Shared State
            self.sensor_state["obstacle_front"] = (dist_front < self.STEP_SIZE)
           # self.sensor_state["obstacle_right"] = (dist_right < self.STEP_SIZE)
           # self.sensor_state["obstacle_left"] = (dist_left < self.STEP_SIZE)

            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_log_time > self.LOG_INTERVAL:
                self.last_log_time = now
                
                self.get_logger().info(
                    f"[THROTTLED] Distances — Front:{dist_front:.2f} m, "
                #   f"Right:{dist_right:.2f} m, Left:{dist_left:.2f} m"
            )
            #self.get_logger().info(f"[THROTTLED] Sensor State: {self.sensor_state}")

            if self.sensor_state["obstacle_front"]:
                self.get_logger().info(f"[THROTTLED] FRONT OBSTACLE DETECTED at {dist_front:.2f}m")
            #if self.sensor_state["obstacle_right"]:
             #   self.get_logger().info(f"[THROTTLED] RIGHT OBSTACLE DETECTED at {dist_right:.2f}m")
           # if self.sensor_state["obstacle_left"]:
            #    self.get_logger().info(f"[THROTTLED] LEFT OBSTACLE DETECTED at {dist_left:.2f}m")
            # log if obstacle detected
            #self.get_logger().info(f"Distances - Front: {dist_front:.2f} m, Right: {dist_right:.2f} m, Left: {dist_left:.2f} m")
            #if self.sensor_state["obstacle_front"]:
             #   self.get_logger().info(f"Obstacle detected! Front: {dist_front:.2f} m")
            #if self.sensor_state["obstacle_right"]:
             #   self.get_logger().info(f"Obstacle detected! Right: {dist_right:.2f} m")
            #if self.sensor_state["obstacle_left"]:
             #   self.get_logger().info(f"Obstacle detected! Left: {dist_left:.2f} m")
            #self.get_logger().info(f"Sensor State: {self.sensor_state}")
            
        except RuntimeError as e:
            self.get_logger().error(f"FAILED to start OAK-D: {e}")
            raise e
    # def on_depth(self, msg: Image):
    #     try:
    #         depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    #     except Exception as e:
    #         self.get_logger().error(f"Depth conversion failed: {e}")
    #         return
    #
    #     depth_m = depth.astype(np.float32) / 1000.0
    #     self.process_depth(depth_m)

    # def process_depth(self, depth):
    #     h, w = depth.shape
    #     ch = int(h * self.crop_ratio)
    #     cw = int(w * self.crop_ratio)
    #     y0 = (h - ch) // 2
    #     x0 = (w - cw) // 2
    #     roi = depth[y0:y0+ch, x0:x0+cw]
    #
    #     valid = roi[roi > 0.05]
    #     if valid.size == 0:
    #         return
    #
    #     median_m = float(np.median(valid))
    #
    #     if median_m < self.thresh_m:
    #         self.obstacle_count += 1
    #         self.clear_count = 0
    #     else:
    #         self.clear_count += 1
    #         self.obstacle_count = 0
    #
    #     if self.obstacle_count >= self.obstacle_frames_needed and self.last_state != 'turn_left':
    #         self.last_state = 'turn_left'
    #         self.pub.publish(String(data='turn_left'))
    #         self.get_logger().info(f"Obstacle detected (median={median_m:.2f} m) -> turn_left")
    #
    #     elif self.clear_count >= self.clear_frames_needed and self.last_state == 'turn_left':
    #         self.last_state = 'clear'
    #         self.pub.publish(String(data='clear'))
    #         self.get_logger().info("Path clear -> resume")
    #
    # def on_color(self, msg: Image):
    #     frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #
    #     lower_yellow = np.array([10, 100, 100])
    #     upper_yellow = np.array([35, 255, 255])
    #     mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #
    #     yellow_percent = (np.count_nonzero(mask) / mask.size) * 100.0
    #     self.get_logger().info(f"Yellow percent: {yellow_percent:.2f}")
    #
    #     if yellow_percent > self.yellow_percent_threshold:
    #         self.yellow_count += 1
    #     else:
    #         self.yellow_count = 0
    #
    #     if self.yellow_count >= self.yellow_frames_needed and self.last_state != 'STOP_YELLOW':
    #         self.last_state = 'STOP_YELLOW'
    #         self.pub.publish(String(data='STOP_YELLOW'))
    #         self.get_logger().info("Yellow detected -> stop")
    #
    #     elif yellow_percent < self.yellow_percent_threshold and self.last_state == 'STOP_YELLOW':
    #         self.last_state = 'clear'
    #         self.pub.publish(String(data='clear'))
    #         self.get_logger().info("Yellow cleared -> resume")


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
