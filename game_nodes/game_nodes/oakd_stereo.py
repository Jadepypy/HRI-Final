import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import numpy as np

class OakdStereoNode(Node):
    def __init__(self):
        super().__init__('oakd_stereo')
        self.get_logger().info("Starting OAK-D Stereo (Modern Gen2 API)...")

        self.bridge = CvBridge()
        self.depth_pub = self.create_publisher(Image, "/stereo/converted_depth", 10)

        # --- Pipeline ---
        pipeline = dai.Pipeline()

        # Create nodes
        monoL = pipeline.create(dai.node.MonoCamera)
        monoR = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        xout_depth = pipeline.create(dai.node.XLinkOut)

        xout_depth.setStreamName("depth")

        # --- Camera Config ---
        monoL.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoR.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # --- Stereo Config (Modern) ---
        # This line crashed before, but will work after the update:
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        
        stereo.setLeftRightCheck(True)
        # We DO NOT set alignment here to avoid "Not Implemented" warnings.
        # It defaults to Rectified Right, which is perfect.

        # --- Linking ---
        monoL.out.link(stereo.left)
        monoR.out.link(stereo.right)
        stereo.depth.link(xout_depth.input)

        # --- Start Device ---
        try:
            self.device = dai.Device(pipeline)
            
            # USB Speed Check
            usb_speed = self.device.getUsbSpeed()
            self.get_logger().info(f"OAK-D connected via: {usb_speed}")
            
            self.q_depth = self.device.getOutputQueue("depth", maxSize=1, blocking=False)
            self.create_timer(0.05, self.publish_frames)
            
        except RuntimeError as e:
            self.get_logger().error(f"FAILED to start OAK-D: {e}")
            raise e

    def publish_frames(self):
        try:
            in_depth = self.q_depth.tryGet()
            if in_depth:
                frame = in_depth.getFrame()
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="16UC1")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "oak_stereo_frame"
                self.depth_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Frame processing error: {e}")

    def destroy_node(self):
        if hasattr(self, 'device'):
            self.device.close()
        super().destroy_node()

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
