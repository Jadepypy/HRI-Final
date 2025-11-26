from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import numpy as np
import rclpy


class OakdStereoNode(Node):
    def __init__(self):
        super().__init__('oakd_stereo')

        self.get_logger().info("Starting OAK-D Stereo (Depth Only)...")

        self.bridge = CvBridge()

        # ROS2 publishers (Depth Only)
        self.depth_pub = self.create_publisher(Image, "/stereo/converted_depth", 10)

        # -----------------------------------------------------
        # DepthAI pipeline
        # -----------------------------------------------------
        pipeline = dai.Pipeline()

        # --- Mono Cameras ---
        monoL = pipeline.createMonoCamera()
        monoR = pipeline.createMonoCamera()

        # Use setBoardSocket
        monoL.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoR.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # --- Stereo Depth ---
        stereo = pipeline.createStereoDepth()
        monoL.out.link(stereo.left)
        monoR.out.link(stereo.right)

        # Align depth to the RIGHT mono camera since RGB is gone
        stereo.setDepthAlign(dai.CameraBoardSocket.RIGHT)

        # --- Output Queues ---
        xout_depth = pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # -----------------------------------------------------
        # Start device
        # -----------------------------------------------------
        try:
            self.device = dai.Device(pipeline)
            self.q_depth = self.device.getOutputQueue("depth", maxSize=1, blocking=False)
            self.create_timer(0.05, self.publish_frames)
        except RuntimeError as e:
            self.get_logger().error(f"FAILED to start OAK-D: {e}")
            raise e

    def publish_frames(self):
        # -------------- DEPTH ONLY --------------
        in_depth = self.q_depth.tryGet()
        if in_depth:
            frame = in_depth.getFrame()
            # OAK-D depth is uint16 in millimeters
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="16UC1")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.depth_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = OakdStereoNode()
        rclpy.spin(node)
        node.destroy_node()
    except Exception as e:
        print(f"Node crashed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()