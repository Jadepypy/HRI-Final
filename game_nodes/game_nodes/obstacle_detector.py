import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
from geometry_msgs.msg import Twist
import threading



class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()

        # --- Robot Constants ---
        self.STEP_SIZE = 0.5  # meters for obstacle detection
        self.MOVE_SPEED = 0.2
        self.TURN_SPEED = 0.5

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

        # self.thresh_m = 2.0
        # self.crop_ratio = 0.3
        # # self.yellow_percent_threshold = 3
        #
        # self.last_state = 'clear'
        #
        # self.obstacle_frames_needed = 1
        # self.clear_frames_needed = 2
        # self.yellow_frames_needed = 2
        #
        # self.obstacle_count = 0
        # self.clear_count = 0
        # self.yellow_count = 0

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.get_logger().info(
            f"ObstacleDetector running — depth:{self.depth_topic}"
        )

        self.interpreter_thread = threading.Thread(target=self.run_student_program)
        self.interpreter_thread.daemon = True
        self.interpreter_thread.start()
        self.get_logger().info("Scratch Runtime Started")

        # self.create_subscription(Image, self.color_topic, self.on_color, 10)

        # self.pub = self.create_publisher(String, '/obstacle_event', 10)

    def on_depth(self, msg):
        """Fixed: Wider path detection + Sensitivity to off-center obstacles"""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            depth_m = depth.astype(np.float32) / 1000.0
            h, w = depth_m.shape

            # FIX 1: Widen the gaze (0.2 was too narrow, 0.6 covers robot width)
            # Adjust this between 0.5 and 0.8 depending on how fat your robot is.
            path_width_ratio = 0.6 
            crop_w = int(w * path_width_ratio)
            x_start = (w - crop_w) // 2
            
            front_view = depth_m[:, x_start : x_start + crop_w]

            # Filter valid points
            valid_front = front_view[front_view > 0.05]

            if valid_front.size > 0:
                # FIX 2: Use Percentile instead of Median
                # Median (50%) ignores obstacles if they are only on one side of the path.
                # Percentile(5) grabs the closest objects, even if they are off-center.
                dist_front = np.percentile(valid_front, 5)
            else:
                dist_front = 9.9

            # Update State
            self.sensor_state["obstacle_front"] = (dist_front < self.STEP_SIZE)
            
            # Reset side states
            self.sensor_state["obstacle_left"] = False
            self.sensor_state["obstacle_right"] = False

            # Logging
            # now = self.get_clock().now().nanoseconds / 1e9
            # if now - self.last_log_time > self.LOG_INTERVAL:
            #     self.last_log_time = now
            #     self.get_logger().info(f"[THROTTLED] Path ({path_width_ratio*100}%) min dist: {dist_front:.2f}m")
            #
            # if self.sensor_state["obstacle_front"]:
            #     self.get_logger().info(f"[THROTTLED] STOP: Path Blocked at {dist_front:.2f}m")

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

    def run_student_program(self):
        """
        This represents the code structure the student built in the UI.
        In a real app, you would load this from a JSON file.
        """

        # Student Logic: Wall Follower (Right Hand Rule)
        # Defines a SCRIPT (a list of blocks) to run sequentially.
        student_code_ast = [
            {
                "type": "forever",
                "body": [
                    {
                        "type": "if",
                        "condition": "obstacle_front",
                        "then": [
                            {"type": "turn_right"}
                        ],
                        "else": [
                            {"type": "move_forward"}
                        ]
                    }
                ]
            }
        ]
        student_code_ast = [
            {"type": "move_forward"},
            {"type": "turn_left"},
            {"type": "move_forward"},
            {"type": "turn_right"}
        ]

        # Small delay to let sensors warm up
        time.sleep(1.0)
        self.get_logger().info("Starting Student Code...")

        try:
            # Execute the script (list of blocks) sequentially
            # This handles top-level sequential blocks (e.g., "Move", then "Turn", then "Forever")
            for block in student_code_ast:
                self.execute_block(block)
        except Exception as e:
            self.get_logger().error(f"Runtime Error: {e}")
        finally:
            self.stop_robot()

    def execute_block(self, block):
        """Recursive function to run any block"""

        # Safety Check: Stop if ROS is shutting down
        if not rclpy.ok():
            return

        b_type = block.get("type")
        self.get_logger().info(f"execute bock {b_type}")
        # --- CONTROL FLOW BLOCKS ---

        if b_type == "forever":
            while rclpy.ok():
                for sub_block in block.get("body", []):
                    self.execute_block(sub_block)
                # Small sleep to prevent 100% CPU usage
                time.sleep(0.05)

        elif b_type == "if":
            condition_key = block.get("condition")
            # Check sensor state
            if self.sensor_state.get(condition_key, False):
                for sub_block in block.get("then", []):
                    self.execute_block(sub_block)
            else:
                for sub_block in block.get("else", []):
                    self.execute_block(sub_block)

        elif b_type == "until":
            # "Repeat Until <Condition>"
            condition_key = block.get("condition")
            while not self.sensor_state.get(condition_key, False) and rclpy.ok():
                for sub_block in block.get("body", []):
                    self.execute_block(sub_block)
                time.sleep(0.05)

        # --- ACTION BLOCKS ---

        elif b_type == "move_forward":
            self.publish_for_duration(self.MOVE_SPEED, 0.0, 1.8)
        elif b_type == "turn_left":
            self.publish_for_duration(0.0, self.TURN_SPEED, 3.2)
        elif b_type == "turn_right":
            self.publish_for_duration(0.0, -self.TURN_SPEED, 3.2)
        elif b_type == "stop":
            self.publish_cmd(0.0, 0.0)
            time.sleep(0.1)

    # --- ACTUATOR HELPERS ---
    def publish_for_duration(self, linear, angular, duration):
        """
        Publishes the command repeatedly for the given duration.
        This prevents the robot's safety watchdog from stopping the motor.
        """
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok():
            self.publish_cmd(linear, angular)
            # Sleep for 100ms (10Hz publish rate)
            time.sleep(0.1)

        # Stop explicitly at the end of the duration
        self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

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
