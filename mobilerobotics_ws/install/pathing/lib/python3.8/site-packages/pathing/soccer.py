import random
import math
import time
import numpy as np

import matplotlib.pyplot as plt

obstacle_ids = [2,4]
obstacle_radius = 5 #cms
goal_id = 6
max_dist = 15
graph_padding = 10 #cms
instruct_time_lin = 1 #seconds
instruct_time_ang = .2 #seconds
cam_wheel_offset = 7 #cm
wheel_speed = (3.0/4.0)*math.pi

def yaw_from_quaternion(x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

def angle_to_matrix(theta):
	return np.asarray([[math.cos(theta.yaw), -1*math.sin(theta.yaw)]
					    ,[math.sin(theta.yaw), math.cos(theta.yaw)]])

def transformation_from_rot_trans(rotmat, tx, ty):
	return np.block([[rotmat, np.asarray([[tx],[ty]])], [np.zeros((1,2)), 1]])

def transpose_transformation(mat):
	R = np.asarray(mat[0:2, 0:2])
	t = np.transpose(mat[0:2, 2])
	R_trans = np.transpose(R)

	return np.block([[R_trans, np.matmul(-1*R_trans,t)], [np.zeros((1,2)), 1]])


	


	

class LandMark():
	def __init__(self, x, y, rot, needsConversion=False, obs_radius=10):
		self.x = x
		self.y = y
		self.vector = np.asarray([x, y, 1]).reshape((3,1))
		self.obs_radius = obs_radius
		if (needsConversion): self.yaw = yaw_from_quaternion(rot[0], rot[2], rot[3], rot[3])
		else: self.yaw = rot

class twoWheelBot(LandMark):
	def __init__(self, length, width, cam_offset):
		super().__init__(0,0,0)
		self.length = length
		self.width = width
		self.cam_offset = cam_offset
		
class Goal():
	def __init__(self, x1, y1, th1, x2, y2):
		self.reference = Landmark(x1,y1,th1)
		x = (x1+x2)/2
		y = (y1+y2)/2
		self.marker = LandMark(x,y,0)
		
	
		
class Point():
	def __init__(self, parent, pos):
		self.parent = parent
		self.children = []
		self.x = pos[0]
		self.y = pos[1]
	
	def addChild(self, child):
		self.children.append(child)
		
class PathPlot():
	def __init__(self, rrt):
		self.rrt = rrt

		


class Smooth():
	def __init__(self, robot, obstacle_list, goal):
		self.robot = robot
		self.obstacle_list = obstacle_list
		#Get landmarks wrt camera
		self.goal_wrt_cam_tf = transformation_from_rot_trans(angle_to_matrix(goal.yaw), goal.x, goal.y)
		self.obstacles_wrt_cam_tfs = [transformation_from_rot_trans(angle_to_matrix(obs.yaw), obs.x, obs.y) for obs in self.obstacle_list]

		#Get bot wrt goal
		self.cam_wrt_goal_tf = transpose_transformation(self.goal_wrt_cam)
		self.bot_wrt_goal_tf = np.matmul(robot.robot_wrt_cam_transformation, self.cam_wrt_goal)

		#Get Obstacles with respect to goal
		self.obstacles_wrt_goal_tfs = [np.matmul(obs_cam, self.cam_wrt_goal) for obs_cam in self.obstacles_wrt_cam_tfs]

		#Rest goal coord to 0,0,0 since everything is based on it
		self.goal = LandMark(0,0,0)
		self.robot.transform_LM(bot_wrt_cam)

		plotter = PathPlot(self)

		
def main(args=None):
	
	robot = twoWheelBot(17, 13, 7)
	obs1 = LandMark(40, 40, math.pi/4)
	obs2 = LandMark(20, 60, math.pi/3)
	ball = LandMark(10, 30, math.pi/-4)
	goal = LandMark(100, 80, math.pi/6)

	pathPlanner = BetterRRT(robot, [obs1, obs2, obs3], goal)
	
if __name__ == '__main__':
	main()			

