import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from aruco_opencv_msgs.msg import ArucoDetection

from rclpy.qos import QoSProfile

import random
import math
import time

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

def euler_from_quaternion(x, y, z, w):
		#THIS FUNCTION WAS TAKEN FROM "automaticaddisom.com/how-to-convert-a-quarternion-into-euler-angles-in-python/"
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        print(yaw_z)
     
        return yaw_z

class LandMark():
	def __init__(self, pos, quarternion=None):
		self.x = pos[0]
		self.y = pos[1]
		self.theta = None
		
		if not quarternion == None:
			self.theta = euler_from_quaternion(quarternion[0], quarternion[2], quarternion[3], quarternion[3])

class Point():
	def __init__(self, parent, pos):
		self.parent = parent
		self.children = []
		self.x = pos[0]
		self.y = pos[1]
	
	def addChild(self, child):
		self.children.append(child)
		



class RRT():
	def __init__(self, root, graph_size, obstacle_list, goal):
		self.root = root
		self.graph_size = graph_size
		#self.x_width = graph_size[0]
		#self.y_width = graph_size[1]
		self.obstacle_list = obstacle_list
		self.goal = goal
		self.final_path = None
		self.x_list = []
		self.y_list = []
		random.seed()
	
	def search_closest(self, x, y):
		#Keep finding the closest child until you come across a Point without a close Point
		current_Point = self.root
		shortest = math.sqrt((self.root.x - x)**2 + (self.root.y - y)**2)
		found_shorter_Point = False
		while(True):
			for child in current_Point.children:
				distance = math.sqrt((child.x - x)**2 + (child.y - y)**2)
				if distance < shortest:
					shortest = distance
					current_Point = child
					found_shorter_Point = True
					break		
			if (not found_shorter_Point):
				return current_Point
			found_shorter_Point = False
	
	def check_goal_dist(self, pos):
		distance = math.sqrt((self.goal.x - pos[0])**2 + (self.goal.y - pos[1])**2)
		if distance <= 10 + cam_wheel_offset:
			return True
		return False	
			
	def addPoint(self):
		#print("ADD POINT")
		#Create random continuous point inside bounding box
		randx = random.uniform(self.graph_size[1], self.graph_size[0])
		randy = random.uniform(self.graph_size[3], self.graph_size[2])
		
		#Find the closest node
		closest = self.search_closest(randx, randy)
		
		#Use max length to find where to put the new node on the shortest path
		new_x = None
		new_y = None
		distance = math.sqrt((randx - closest.x)**2 + (randy - closest.y)**2)
		if distance > max_dist:
			print("TOO FAR")
			#MAY HAVE DONE THIS MATH WRONG. I NEED INVERSE TRIG
			slope = (randy - closest.y)/(randx - closest.x)
			theta = math.tan(slope)
			delta_x = max_dist*math.cos(theta)
			delta_y = max_dist*math.sin(theta)
			new_x = closest.x + delta_x
			new_y = closest.y + delta_y
		else:
			new_x = randx
			new_y = randy
		
		print(str((new_x,new_y)))	
		
		#Use this to visualize new nodes as they show up
		#self.x_list.append(new_x)
		#self.y_list.append(new_y)
		#plt.scatter(self.x_list, self.y_list)
		#plt.show()
		
		#Check to see if there is an obstacle between the closest node and the new node
		#First put path from closest to new in standard form
		#y - closest.y = slope*(x - closest.x) -> ax + by + c = 0
		#multiply everything by (new_x - closest.x)
		a = new_y - closest.y
		b = new_x - closest.x
		c = -(a*closest.x) - (b*closest.y)
		placable = True
		for obstacle in self.obstacle_list:
			if a > 0:
				max_y = new_y + obstacle_radius
				min_y = closest.y - obstacle_radius
			else:
				max_y = closest.y + obstacle_radius
				min_y = new_y - obstacle_radius
			if b > 0:
				max_x = new_x + obstacle_radius
				min_x = closest.x - obstacle_radius
			else:
				max_x = closest.x + obstacle_radius
				min_x = new_x - obstacle_radius	
				
			if (obstacle.x > min_x and obstacle.x < max_x and obstacle.y > min_y and obstacle.y < max_y):
				#This is in the space that could block the path
				#check to see if the distance from the shortest path to the obstacle is less than the radius of the obstacle
				distance = (abs(a*obstacle.x + b*obstacle.y + c)/(math.sqrt(a**2 + b**2)))
				if distance > obstacle_radius:
					#CANNOT PLACE POINT
					placable = False
					print("OBSTACLE")
					break
					
		#If there was no obstacle in the way, the place the new point down			
		if placable:
			newPoint = Point(closest, (new_x, new_y))
			closest.addChild(newPoint)
			print("NEW POINT")
			#Now check to see if this point is within 10 cm to the goal
			if self.check_goal_dist((new_x, new_y)):
				#Create Final Path
				#Go from newPoint all the way back from Parent to Parent until root
				current_point = newPoint
				self.final_path = []
				self.final_path.append(newPoint)
				while (not current_point.parent == None):
					print((current_point.x, current_point.y))
					self.final_path.append(current_point.parent)
					current_point = current_point.parent
					
				self.final_path = self.final_path[::-1]	
				print((self.final_path[0].x, self.final_path[0].y))
				
		
		 
		

class Planner(Node):
	def __init__(self):
		super().__init__('planner')
		self.subscription = self.create_subscription(ArucoDetection, 'aruco_detections', self.detection_callback, 10)
		self.subscription
		qos = QoSProfile(depth=10)
		self.pub = self.create_publisher(Twist, '/jetbot/cmd_vel', qos)
		self.rrt = None
		
		self.lin_vel = 0
		self.ang_vel = 0
		
		self.dirTurn = 1
		self.delta_theta = 0
		
		
	def twist(self, direction, delta, typeMotion):
		
		sleepTime = 0
		if typeMotion == "lin":
			self.ang_vel = 0
			self.lin_vel = wheel_speed/4
			sleepTime = 4*(delta/100.0)/wheel_speed
		else:
			self.lin_vel = 0
			self.ang_vel = wheel_speed
			sleepTime = (delta/wheel_speed)*(2.0/3.0)
				
			self.lin_vel *= direction
			self.ang_vel *= direction
		
		#MOVE
		twist = Twist()
		twist.linear.x = -1.0*float(self.lin_vel)
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = float(self.ang_vel)
		
		print("Linear: " + str(-1.0*float(self.lin_vel)/100.0) + ", Angular: " + str(float(self.ang_vel)*(2.0/3.0)) + ", delta: " + str(direction*delta))
		
		self.pub.publish(twist)
		
		time.sleep(sleepTime)
		
		#STOP MOVING
		twist = Twist()
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0
		
		self.pub.publish(twist)
		
	def detection_callback(self, msg):
		#self.get_logger().info(str(msg))
		
		if self.rrt == None and len(msg.markers) == 3:
			#create rrt
			#first make obstacle list 
			obstacle_list = []
			goal = None
			min_x = None
			min_y = -1*cam_wheel_offset
			max_x = None
			max_y = None
			for marker in msg.markers:
				#print(marker)
				if marker.marker_id in obstacle_ids:
					#Landmark is an obstacle
					print("OBSTACLE")
					obstacle_list.append(LandMark((marker.pose.position.x*100.0, marker.pose.position.z*100.0)))
					if min_x == None or marker.pose.position.x*100 - graph_padding < min_x:
						min_x = marker.pose.position.x*100 - graph_padding
					if max_x == None or marker.pose.position.x*100 + graph_padding > max_x:
						max_x = marker.pose.position.x*100 + graph_padding
				elif marker.marker_id == goal_id:
					#Landmark is the goal
					print("GOAL")
					position = marker.pose.position
					orientation = marker.pose.orientation
					goal = LandMark((position.x*100.0, position.z*100.0), quarternion=(orientation.x, orientation.y, orientation.z, orientation.w))
					max_y = marker.pose.position.z*100.0 + graph_padding
					print(goal.theta)
			print("minx maxx miny maxy: " + str((min_x, max_x, min_y, max_y)))
				
			graph_size = (max_x, min_x, max_y, min_y)
			root = Point(parent=None, pos=(0, min_y))
			#$print(str(graph_size))
			print(str((root.x, root.y)))
			
			self.rrt = RRT(root, graph_size, obstacle_list, goal)
			print("Created RRT")
		
			#Now add points until final_path is created
			while (self.rrt.final_path == None):
				print("Adding point?")
				self.rrt.addPoint()
			
			print("Done adding Points, final path found")
			print("OBSTACLES:")
			for obstacle in self.rrt.obstacle_list:
				print(str((obstacle.x, obstacle.y)))
			print("GOAL:")	
			print(str((self.rrt.goal.x, self.rrt.goal.y)))
			
			#SEND INIT TWIST OR ELSE IT DOESNT WORK FOR SOME REASON
			self.twist(1, 0, "ang")
			time.sleep(3)
			#For the pathing to work, we need to force the initial theta to be directly towards the goal
			if goal.theta > 0: #left turn
				self.twist(-1, abs(goal.theta), "ang")	
			else:
				self.twist(1, abs(goal.theta), "ang")		
			
			#Now that there is a final path we need to create a command list
			for i in range(len(self.rrt.final_path)-1):
				#Create motion for every point to point transition
				now = self.rrt.final_path[i]
				next = self.rrt.final_path[i+1]
				
				if next.y > now.y: #Forward
					if next.x > now.x: #angling right
						self.delta_theta = math.atan(abs((next.y - now.y)/(next.x - now.x)))
						self.dirTurn = 1
					else:
						#angling left
						self.delta_theta = math.atan(abs((next.y - now.y)/(next.x - now.x)))
						self.dirTurn = -1
				else: #backward
					if next.x > now.x: #angling right
						self.delta_theta = math.atan(abs((next.y - now.y)/(next.x - now.x))) + math.pi/2.0
						self.dirTurn = 1
					else:
						#angling left
						self.delta_theta = math.atan(abs((next.y - now.y)/(next.x - now.x))) + math.pi/2.0
						self.dirTurn = -1
						
				self.twist(self.dirTurn, self.delta_theta, "ang")
				time.sleep(3)
				forward = math.sqrt((next.x - now.x)**2 + (next.y - now.y)**2)
				self.twist(1, forward, "lin")
				time.sleep(3)
				#turn back forward
				self.twist(-1*self.dirTurn, self.delta_theta, "ang")
				time.sleep(3)

		
		
def main(args=None):
	rclpy.init(args=args)
	
	planner = Planner()
	rclpy.spin(planner)
	
	planner.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()				


