import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import time

class LaneDetectionNode(Node):
    """
    A ROS2 node for lane detection using OpenCV.
    """
    def __init__(self):
        super().__init__('lane_detection_node')
        self.publisher_ = self.create_publisher(Float32, 'lane_offset', 10)