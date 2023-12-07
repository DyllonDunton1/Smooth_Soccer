#First get coords (x,y,th) of everything in aruco subscriber
#use matplotlib to plot all of those in ref to the bot
#do rrt
#plot it
#smooth it
#plot it
#twist it


import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection

from rclpy.qos import QoSProfile

import random
import math
import time
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches


def yaw_from_quaternion(rot):
        t3 = 2.0 * (rot.w * rot.z + rot.x * rot.y)
        t4 = 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
        return math.atan2(t3, t4) - math.pi/2

class LandMark():
	def __init__(self, x, y, rot, needsConversion=False, obs_radius=20):
		self.x = x
		self.y = y
		self.vector = np.asarray([x, y, 1]).reshape((3,1))
		self.obs_radius = obs_radius
		if (needsConversion): self.yaw = yaw_from_quaternion(rot)
		else: self.yaw = rot


		
class Goal():
	def __init__(self, x1, y1, th1, x2, y2):
		self.reference = LandMark(x1,y1,th1, needsConversion=True)
		print("GOAL ANGLE: {}".format(self.reference.yaw))
		x = (x1+x2)/2
		y = (y1+y2)/2
		self.marker = LandMark(x,y,0)
		
class Point(LandMark):
	def __init__(self, parent, x, y):
		super().__init__(x,y,0)
		self.parent = parent
		self.children = []
		
	
	def addChild(self, child, points):
		self.children.append(child)
		points.append(child)
		
class twoWheelBot(Point):
	def __init__(self, length, width, cam_offset):
		super().__init__(0,0,0)
		self.length = length
		self.width = width
		self.cam_offset = cam_offset


class RRT():
	def __init__(self, robot, obstacle_list, ball, goal, plot=True):
		
		self.obs = obstacle_list
		self.ball = ball
		self.reference = goal.reference
		self.goal = goal.marker
		self.root = Point(None, robot.x, robot.y)
		self.points = [self.root]
		self.plot = plot
		
		robot_yaw = math.atan2(self.obs[0].y, self.obs[0].x)
		
		
		self.bot = LandMark(0, 0, robot_yaw)
		
		self.max_dist = 5 #chose this because this is the obstacle radius needs to be much bigger
		self.shoot_dist = 25
		
		graph_pad = 50
		
		#make graph size
		self.max_x = max(self.root.x, *[o.x for o in self.obs], ball.x, goal.reference.x, goal.marker.x) + graph_pad
		self.min_x = min(self.root.x, *[o.x for o in self.obs], ball.x, goal.reference.x, goal.marker.x) - graph_pad
		self.max_y = max(self.root.y, *[o.y for o in self.obs], ball.y, goal.reference.y, goal.marker.y) + graph_pad
		self.min_y = min(self.root.y, *[o.y for o in self.obs], ball.y, goal.reference.y, goal.marker.y) - graph_pad
		print("Min X: {}".format(self.min_x))
		print("Max X: {}".format(self.max_x))
		print("Min Y: {}".format(self.min_y))
		print("Max Y: {}".format(self.max_y))
		
		
		#find shooting position
		theta = math.atan2(self.goal.y - ball.y, self.goal.x - ball.x)
		self.shoot = LandMark(ball.x - self.shoot_dist*math.cos(theta), ball.y - self.shoot_dist*math.sin(theta), theta)
		
		self.final_path = None
		
		random.seed()
		
		if self.plot:
			self.fig, self.ax = plt.subplots()
			self.ax.set_xlim(self.min_x, self.max_x)
			self.ax.set_ylim(self.min_y, self.max_y)
			
			self.ax.arrow(self.root.x, self.root.y, 5*np.cos(self.bot.yaw), 5*np.sin(self.bot.yaw), width=1, color="red")
			for o in self.obs:
				self.ax.add_patch(patches.Rectangle((o.x, o.y), 2, 2, linewidth=1, edgecolor='r', facecolor='r'))
			self.ax.add_patch(patches.Rectangle((self.reference.x, self.reference.y), 2, 2, linewidth=1, edgecolor='b', facecolor='b'))
			self.ax.add_patch(patches.Rectangle((self.goal.x, self.goal.y), 2, 2, linewidth=1, edgecolor='g', facecolor='g'))
			self.ax.add_patch(patches.Rectangle((self.ball.x, self.ball.y), 2, 2, linewidth=1, edgecolor='y', facecolor='y'))
			self.ax.add_patch(patches.Rectangle((self.shoot.x, self.shoot.y), 2, 2, linewidth=1, edgecolor='y', facecolor='y'))
		

	
	def search_closest(self, x, y):
		#time.sleep(4)
		closest = self.root
		dist = 1000000
		#print("------------------------")
		for point in self.points:
			#print("X: {}, Y: {}".format(point.x, point.y))
			distance = math.sqrt((point.x - x)**2 + (point.y - y)**2)
			if distance < dist:
				closest = point
				dist = distance
				#print("new")
		#at this point we have found the closest point
		#print("------------------------")
		return closest	
		
	def too_close_to_obs(self, x, y):
		obstacles = [*self.obs, self.ball, self.reference, self.goal]
		for o in obstacles:
			dist = math.sqrt((o.x - x)**2 + (o.y - y)**2)	
			if dist < o.obs_radius:
				return True
		return False			
	
	def check_shoot_dist(self, x, y):
		distance = math.sqrt((self.shoot.x - x)**2 + (self.shoot.y - y)**2)
		if distance <= 10:
			return True
		return False	
			
	def addPoint(self):
		#print("ADD POINT")
		#Create random continuous point inside bounding box
		randx = random.uniform(self.min_x, self.max_x)
		randy = random.uniform(self.min_y, self.max_y)
		#print(randx, randy)
		#Find the closest node
		closest = self.search_closest(randx, randy)
		
		#Use max length to find where to put the new node on the shortest path
		new_x = None
		new_y = None
		distance = math.sqrt((randx - closest.x)**2 + (randy - closest.y)**2)
		if distance > self.max_dist:
			#print("TOO FAR")
			#MAY HAVE DONE THIS MATH WRONG. I NEED INVERSE TRIG
			theta = math.atan2(randy - closest.y, randx - closest.x)
			delta_x = self.max_dist*math.cos(theta)
			delta_y = self.max_dist*math.sin(theta)
			new_x = closest.x + delta_x
			new_y = closest.y + delta_y
			#print(new_x, new_y)
		else:
			new_x = randx
			new_y = randy
		
		#print(str((new_x,new_y)))	
		
		
		if not self.too_close_to_obs(new_x, new_y): #actually place if not too close
		
			
			#If there was no obstacle in the way, the place the new point down			
			newPoint = Point(closest, new_x, new_y)
			closest.addChild(newPoint, self.points)
			if self.plot: self.ax.plot([closest.x, new_x], [closest.y, new_y], color="black", linewidth=2, markersize=3)
			#plt.show()
			print("NEW POINT")
			#Now check to see if this point is within 10 cm to the goal
			if self.check_shoot_dist(new_x, new_y):
				#Create Final Path
				#Go from newPoint all the way back from Parent to Parent until root
				current_point = newPoint
				self.final_path = [self.shoot]
				self.final_path.append(newPoint)
				while (not current_point.parent == None):
					self.final_path.append(current_point.parent)
					current_point = current_point.parent
				self.final_path = self.final_path[::-1]	
				print("-------------------")
				print("LENGTH = {}".format(len(self.final_path)))
				for point in self.final_path:
					print("X: {}, Y: {}".format(point.x, point.y))
				
				if self.plot:
					for i in range(len(self.final_path)-1):
						start = self.final_path[i]
						end = self.final_path[i+1]
						self.ax.plot([start.x, end.x], [start.y, end.y], color="yellow", linewidth=2, markersize=3)
					#plt.show()
				#NOW APPLY SMOOTHING!
				self.starting_smooth = 0
				self.new_walkable = self.starting_smooth + 2

				self.poppers = []
				while True:
					print("BREAKING AT NW={}".format(self.new_walkable))
					if self.new_walkable >= len(self.final_path):
						break
					start = self.final_path[self.starting_smooth]
					end = self.final_path[self.new_walkable]
					a = end.y - start.y
					b = start.x - end.x
					c = start.y*(end.x-start.x) - (end.y-start.y)*start.x
					for o in self.obs:
						
						x_dist = (end.x - start.x)
						m = (end.y - start.y)/x_dist
						b = start.y - start.x*m
						points = 25
						x_dist_split = x_dist/25
						split_points = []
						for i in range(points):
							x_val = start.x + i*x_dist_split
							y_val = m*x_val + b
							split_points.append(Point(None, x_val, y_val))
						dists = [math.sqrt((p.x - o.x)**2 + (p.y - o.y)**2) for p in split_points]
						if min(dists) < o.obs_radius:
							#TOO CLOSE, NOT WALKABLE
							print("CUTTING")
							print("start: {}".format(self.starting_smooth))
							print("end: {}".format(self.new_walkable))
							print("-------------------")
							for i in range(self.new_walkable - self.starting_smooth - 2):
								self.poppers.append(i+self.starting_smooth + 1)
								
							self.starting_smooth = self.new_walkable-1	
							break
						if self.new_walkable == len(self.final_path)-1:
							for i in range(self.new_walkable - self.starting_smooth - 1):
								self.poppers.append(i+self.starting_smooth + 1)
							break	
					self.new_walkable += 1
				print(str(self.poppers))
				for index in sorted(self.poppers, reverse=True):
					del self.final_path[index]	
				print("-------------------")
				for point in self.final_path:
					print("X: {}, Y: {}".format(point.x, point.y))		
				if self.plot:
					for i in range(len(self.final_path)-1):
						start = self.final_path[i]
						end = self.final_path[i+1]
						self.ax.plot([start.x, end.x], [start.y, end.y], color="red", linewidth=2, markersize=3)
				self.travel_stops = [self.bot]
				self.distances = []
				for stop in range(len(self.final_path) - 1):
					now = self.final_path[stop]
					next = self.final_path[stop + 1]
					yaw = math.atan2(next.y - now.y, next.x - now.x)
					lm = LandMark(now.x, now.y, yaw)
					self.distances.append(math.sqrt((next.x - now.x)**2 + (next.y - now.y)**2))
					self.travel_stops.append(lm)
					if self.plot: self.ax.arrow(lm.x, lm.y, 5*np.cos(lm.yaw), 5*np.sin(lm.yaw), width=1, color="blue")
				now = self.final_path[-1]
				next = self.goal
				yaw = math.atan2(next.y - now.y, next.x - now.x)
				lm = LandMark(now.x, now.y, yaw)
				self.travel_stops.append(lm)
				self.distances.append(1.25*math.sqrt((next.x - now.x)**2 + (next.y - now.y)**2))
				print(str([x.yaw for x in self.travel_stops]))
				print(str(self.distances))
				if self.plot: self.ax.arrow(lm.x, lm.y, 5*np.cos(lm.yaw), 5*np.sin(lm.yaw), width=1, color="blue")
				if self.plot: plt.show()
				
				
				
class SmoothSoccer(Node):
	def __init__(self):
		super().__init__('robot')
		self.subscription = self.create_subscription(ArucoDetection, 'aruco_detections', self.detection_callback, 10)
		self.subscription
		qos = QoSProfile(depth=10)
		self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', qos)
		
		self.turn_speed = (3.0/4.0)*math.pi
		self.forw_speed = 0.1
		self.rot_mult = .32
		self.lin_mult = 6.7
		
		self.plot = True
		
		self.rrt = None
		self.reset()
		

	def detection_callback(self, msg):
		if self.rrt == None and len(msg.markers) == 5:
			obs1 = None
			obs2 = None
			ball = None
			goal = None
			g1 = None
			g2 = None
			for marker in msg.markers:
				id_ = marker.marker_id
				x = marker.pose.position.x*86.2069 - 1.37931
				y = marker.pose.position.z*168.919 + 2.195
				q = marker.pose.orientation
				if id_ == 0: #GOAL REFERENCE
					g1 = (x, y, q)
				if id_ == 4: #GOAL 2
					g2 = (x, y)
				if id_ == 2: #Obs 1
					obs1 = LandMark(x,y+10,0)
				if id_ == 3: #Obs 2
					obs2 = LandMark(x,y,0)
				if id_ == 1: #Ball
					ball = LandMark(x,y,0)			
					
					
			robot = twoWheelBot(17, 13, 7)
			goal = Goal(*g1, *g2)
			obs = [obs1, obs2]		
			
			self.rrt = RRT(robot, obs, ball, goal, plot=self.plot)
			
			while (self.rrt.final_path == None):
				print("Adding point?")
				self.rrt.addPoint()
			print("Done adding Points, final path found")
			
			
			for i in range(len(self.rrt.travel_stops) - 1):
				self.turn(self.rrt.travel_stops[i+1].yaw - self.rrt.travel_stops[i].yaw)
				self.forward(self.rrt.distances[i])
			
			

	def turn(self, angle_cw):
		if angle_cw < 0: sign = -1
		else: sign = 1
		
		print("TURNING {} radians".format(angle_cw))
		twist = Twist()
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = -1.0*float(self.turn_speed)*sign
		self.pub.publish(twist)
		time.sleep(self.rot_mult*abs(angle_cw))
		self.reset()
	
	def forward(self, distance):
		print("FORWARD {} meters".format(distance/100))
		twist = Twist()
		twist.linear.x = -1.0*float(self.forw_speed)
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = -0.12 #Counteract drift
		
		self.pub.publish(twist)
		time.sleep(self.lin_mult*distance/100)
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
		
		
		
def main(args=None):
	rclpy.init(args=None)
	
	soccer = SmoothSoccer()
	rclpy.spin(soccer)
	
	soccer.destroy_node()
	rclpy.shutdown()
	
	
	
	
if __name__ == '__main__':
	main()	
