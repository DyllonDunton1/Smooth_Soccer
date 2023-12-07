import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection

from rclpy.qos import QoSProfile

import random
import math
import time
import numpy as np

class Robot(Node):
	def __init__(self):
		super().__init__('robot')
		qos = QoSProfile(depth=10)
		self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', qos)
		self.turn_speed = (3.0/4.0)*math.pi
		self.forw_speed = .1
		self.rot_mult = 1/3
		self.lin_mult = 6.775
		self.reset()
		
		while True:
			self.turn(math.pi/2)
			time.sleep(5)


	def turn(self, angle_cw):
		if angle_cw < 0: angle_cw += 2*math.pi
		print("TURNING {} radians".format(angle_cw))
		twist = Twist()
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = -1.0*float(self.turn_speed)
		self.pub.publish(twist)
		time.sleep(self.rot_mult*angle_cw)
		self.reset()
	
	def forward(self, distance):
		print("FORWARD {} meters".format(distance))
		twist = Twist()
		twist.linear.x = -1.0*float(self.forw_speed)
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = -0.1 #Counteract drift
		
		self.pub.publish(twist)
		time.sleep(self.lin_mult*distance)
		self.reset()
	
	def reset(self):
		twist = Twist()
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0
		
		self.pub.publish(twist)
		
	
	def twist(self, action):
		print("Twist")
		twist = Twist()
		lin = None
		ang = None
		if action == 0: #forward
			lin = self.wheel_speed
			ang = 0
		if action == 1: #left
			lin = 0
			ang = -self.wheel_speed
		if action == 2: #right
			lin = 0
			ang = self.wheel_speed


		twist.linear.x = -1.0*float(lin)
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = float(ang)
		
		self.pub.publish(twist)
		time.sleep(1)


def main(args=None):
	rclpy.init(args=None)
	
	robot = Robot()
	rclpy.spin(robot)
	
	robot.destroy_node()
	rclpy.shutdown()
