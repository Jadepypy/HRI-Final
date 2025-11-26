import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import threading
import time


class ScratchRuntime(Node):

    def __init__(self):
        super().__init__('scratch_runtime')
        self.bridge = CvBridge()

        # --- Robot Constants ---
        self.STEP_SIZE = 0.5  # meters for obstacle detection
        self.MOVE_SPEED = 0.2
        self.TURN_SPEED = 0.5

        # --- Sensor State (Shared Memory) ---
        # The ROS callbacks update these, the Interpreter reads them.
        self.sensor_state = {
            "obstacle_right": False,
            "obstacle_front": False,
            "obstacle_left": False
        }
        self.latest_depth = None

        # --- ROS Setup ---
        # self.create_subscription(Image, '/stereo/converted_depth', self.on_depth, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Start the Interpreter in a Background Thread ---
        # This allows the logic to have blocking loops (forever, sleep)
        # without freezing the ROS sensor callbacks.
        self.interpreter_thread = threading.Thread(target=self.run_student_program)
        self.interpreter_thread.daemon = True
        self.interpreter_thread.start()

        self.get_logger().info("Scratch Runtime Started")

    # def on_depth(self, msg):
    #     """Standard Perception Logic from before"""
    #     try:
    #         depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
    #         depth_m = depth.astype(np.float32) / 1000.0
    #         h, w = depth_m.shape
    #         col_w = w // 3
    #
    #         # Slice Regions
    #         # Front (Center)
    #         front_view = depth_m[:, col_w:2 * col_w]
    #         valid_front = front_view[front_view > 0.05]
    #         dist_front = np.median(valid_front) if valid_front.size > 0 else 9.9
    #
    #         # Right
    #         right_view = depth_m[:, 2 * col_w:]
    #         valid_right = right_view[right_view > 0.05]
    #         dist_right = np.median(valid_right) if valid_right.size > 0 else 9.9
    #
    #         # Left
    #         left_view = depth_m[:, :col_w]
    #         valid_left = left_view[left_view > 0.05]
    #         dist_left = np.median(valid_left) if valid_left.size > 0 else 9.9
    #
    #         # Update Shared State
    #         self.sensor_state["obstacle_front"] = (dist_front < self.STEP_SIZE)
    #         self.sensor_state["obstacle_right"] = (dist_right < self.STEP_SIZE)
    #         self.sensor_state["obstacle_left"] = (dist_left < self.STEP_SIZE)
    #
    #     except Exception:
    #         pass

    # ==========================================
    #      THE INTERPRETER (Runtime Engine)
    # ==========================================

    def run_student_program(self):
        """
        This represents the code structure the student built in the UI.
        In a real app, you would load this from a JSON file.
        """

        # Student Logic: Wall Follower (Right Hand Rule)
        # Defines a SCRIPT (a list of blocks) to run sequentially.
        # student_code_ast = [
        #     {
        #         "type": "forever",
        #         "body": [
        #             {
        #                 "type": "if",
        #                 "condition": "obstacle_right",
        #                 "then": [
        #                     # Sequence inside IF: Check front, then maybe stop/turn
        #                     {
        #                         "type": "if",
        #                         "condition": "obstacle_front",
        #                         "then": [
        #                             {"type": "stop"},
        #                             {"type": "turn_left"}
        #                         ],
        #                         "else": [
        #                             {"type": "move_forward"}
        #                         ]
        #                     }
        #                 ],
        #                 "else": [
        #                     {"type": "turn_right"}
        #                 ]
        #             }
        #         ]
        #     }
        # ]
        student_code_ast = [
            {"type": "move_forward"},
            #{"type": "move_forward"},
            #{"type": "move_forward"},
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
    node = ScratchRuntime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
