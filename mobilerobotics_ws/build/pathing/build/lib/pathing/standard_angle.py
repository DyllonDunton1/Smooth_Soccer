import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection

from rclpy.qos import QoSProfile

import random
import math
import time
import numpy as np


def yaw_from_quaternion(rot):
        t2 = +2.0 * (rot.w * rot.y - rot.z * rot.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        return math.asin(t2) * 180 / math.pi


class Robot(Node):
	def __init__(self):
		super().__init__('robot')
		self.subscription = self.create_subscription(ArucoDetection, 'aruco_detections', self.detection_callback, 10)
		self.subscription
		qos = QoSProfile(depth=10)

	def detection_callback(self, msg):
		for marker in msg.markers:
			id_ = marker.marker_id
			x = marker.pose.position.x*86.2069 - 1.37931
			y = marker.pose.position.z*168.919 + 2.195
			q = marker.pose.orientation
			print("------------------------------")
			print("X: {}".format(x))
			print("Y: {}".format(y))
			print("Theta: {}".format(yaw_from_quaternion(q)))
					
					
					


def main(args=None):
	rclpy.init(args=None)
	
	robot = Robot()
	rclpy.spin(robot)
	
	robot.destroy_node()
	rclpy.shutdown()
