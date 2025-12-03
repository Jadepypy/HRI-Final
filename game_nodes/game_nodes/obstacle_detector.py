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
from flask import Flask, request, jsonify
from flask_cors import CORS

app = Flask(__name__)

# ENABLE CORS: Allow all domains (*) to access all routes (/*)
CORS(app, resources={r"/*": {"origins": "*"}})
robot_node = None  # Global reference so Flask can talk to ROS


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()

        # --- Robot Constants ---
        self.STEP_SIZE = 0.5
        self.MOVE_SPEED = 0.2
        self.TURN_SPEED = 0.5

        self.sensor_state = {
            "obstacle_right": False,
            "obstacle_front": False,
            "obstacle_left": False
        }

        self.depth_topic = '/stereo/converted_depth'
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)

        self.get_logger().info(f"ObstacleDetector running — depth:{self.depth_topic}")

        # --- Thread Control Flags ---
        self.is_running = False
        self.current_thread = None

    def on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            depth_m = depth.astype(np.float32) / 1000.0
            h, w = depth_m.shape

            path_width_ratio = 0.6
            crop_w = int(w * path_width_ratio)
            x_start = (w - crop_w) // 2

            front_view = depth_m[:, x_start: x_start + crop_w]
            valid_front = front_view[front_view > 0.05]

            if valid_front.size > 0:
                dist_front = np.percentile(valid_front, 5)
            else:
                dist_front = 9.9

            self.sensor_state["obstacle_front"] = (dist_front < self.STEP_SIZE)
            
            #if self.sensor_state["obstacle_front"]:
            #    self.get_logger().info("in front!")
            self.sensor_state["obstacle_left"] = False
            self.sensor_state["obstacle_right"] = False

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")

    # ---------------------------------------------------------
    # THREAD LIFECYCLE (FIXED)
    # ---------------------------------------------------------

    def start_student_program(self, code_json):
        self.get_logger().info("Request to START...")

        # 1. Signal Stop
        self.stop_student_program()

        # 2. Wait for old thread
        if self.current_thread is not None and self.current_thread.is_alive():
            self.current_thread.join(timeout=3.0)  # Give it 3 seconds

            # CRITICAL SAFETY CHECK
            if self.current_thread.is_alive():
                self.get_logger().error("CRITICAL: Old thread is STUCK. Cannot start new program.")
                # We return here to prevent the 'Two Driver' crash
                # The user will see that the robot didn't start, which is safer than it going crazy.
                return

                # 3. Only if the coast is clear, start the new one
        self.is_running = True
        self.current_thread = threading.Thread(target=self.interpreter_loop, args=(code_json,))
        self.current_thread.daemon = True
        self.current_thread.start()
        self.get_logger().info("New Program Thread Started.")

    def stop_student_program(self):
        self.is_running = False  # Signal the thread to stop
        self.stop_robot()  # Immediate motor halt
        self.get_logger().info("Stop Signal Sent.")

    def interpreter_loop(self, student_code_ast):
        """ The main logic loop that runs the passed JSON """
        self.get_logger().info("Interpreter Loop Started")

        # Wait for connections
        while self.vel_pub.get_subscription_count() == 0:
            if not rclpy.ok() or not self.is_running: return
            time.sleep(0.5)

        try:
            for block in student_code_ast:
                # CHECK: Stop immediately if flag is false
                if not self.is_running: break
                self.execute_block(block)
        except Exception as e:
            self.get_logger().error(f"Runtime Error: {e}")
        finally:
            # CLEANUP: Ensure robot stops and state is clean when thread ends
            self.stop_robot()
            self.is_running = False
            self.get_logger().info("Interpreter Loop Finished")

    # ---------------------------------------------------------
    # EXECUTION LOGIC (FIXED)
    # ---------------------------------------------------------

    def execute_block(self, block):
        """Recursive function to run any block"""

        # CHECK 1: Immediate Exit
        if not self.is_running or not rclpy.ok():
            return

        b_type = block.get("type")

        if b_type == "forever":
            # CHECK 2: Loop Condition must include is_running
            while rclpy.ok() and self.is_running:
                for sub_block in block.get("body", []):
                    if not self.is_running: return  # Break inner loop
                    self.execute_block(sub_block)
                time.sleep(0.05)

        elif b_type == "repeat_until":
            self.get_logger().info("repeat until")
            condition_key = block.get("condition")
            # CHECK 3: Loop Condition must include is_running
            while not self.sensor_state.get(condition_key, False) and rclpy.ok() and self.is_running:
                for sub_block in block.get("body", []):
                    if not self.is_running: return
                    self.execute_block(sub_block)
                time.sleep(0.05)

        elif b_type == "if":
            condition_key = block.get("condition")
            if self.sensor_state.get(condition_key, False):
                for sub_block in block.get("then", []):
                    self.execute_block(sub_block)
            else:
                for sub_block in block.get("else", []):
                    self.execute_block(sub_block)

        elif b_type == "move_forward":
            self.publish_for_duration(self.MOVE_SPEED, 0.0, 0.5)
        elif b_type == "turn_left":
            self.publish_for_duration(0.0, self.TURN_SPEED, 3.2)
        elif b_type == "turn_right":
            self.publish_for_duration(0.0, -self.TURN_SPEED, 3.2)
        elif b_type == "stop":
            self.stop_robot()
            time.sleep(0.1)

    # ---------------------------------------------------------
    # ACTUATOR HELPERS (FIXED)
    # ---------------------------------------------------------

    def publish_for_duration(self, linear, angular, duration):
        """
        Moves robot, but CHECKS self.is_running constantly.
        This fixes the latency issue.
        """
        end_time = time.time() + duration

        # CHECK 4: The Latency Killer
        # We add 'and self.is_running' so the loop breaks instantly on Stop.
        while time.time() < end_time and rclpy.ok() and self.is_running:
            self.publish_cmd(linear, angular)
            time.sleep(0.1)

        self.stop_robot()

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.vel_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)


# --- FLASK ROUTES ---

@app.route('/run', methods=['POST'])
def run_code():
    if not robot_node:
        return jsonify({"error": "Robot initializing"}), 503

    data = request.get_json()
    if not data:
        return jsonify({"error": "No JSON provided"}), 400

    # Start logic in background thread (Non-blocking)
    robot_node.start_student_program(data)
    return jsonify({"status": "started"})


@app.route('/stop', methods=['POST', 'GET'])
def stop_code():
    if robot_node:
        robot_node.stop_student_program()
    return jsonify({"status": "stopped"})


def main(args=None):
    global robot_node
    rclpy.init(args=args)
    robot_node = ObstacleDetector()

    # Start Flask in separate thread
    flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=8000, debug=False, use_reloader=False))
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        pass

    robot_node.stop_robot()
    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
