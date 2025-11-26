#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import depthai as dai
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class OakDStereoNode(Node):
    def __init__(self):
        super().__init__('oakd_stereo_node')

        self.bridge = CvBridge()

        pipeline = dai.Pipeline()

        monoL = pipeline.createMonoCamera()
        monoR = pipeline.createMonoCamera()
        stereo = pipeline.createStereoDepth()

        monoL.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoR.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        monoL.out.link(stereo.left)
        monoR.out.link(stereo.right)

        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)
        stereo.setSubpixel(True)

        xout = pipeline.createXLinkOut()
        xout.setStreamName("depth")
        stereo.depth.link(xout.input)

        self.device = dai.Device(pipeline)
        self.q = self.device.getOutputQueue("depth", 1, False)

        self.pub = self.create_publisher(Image, '/stereo/converted_depth', 10)
        self.timer = self.create_timer(0.03, self.publish_depth)

        self.get_logger().info("OAK-D Stereo node started")

    def publish_depth(self):
        frame = self.q.get().getFrame()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='16UC1')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OakDStereoNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
