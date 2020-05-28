#!/usr/bin/env python

import rospy
import random 
from math import sqrt
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

class RRT():
	def __init__(self):
		rospy.init_node('RRT_node')
		self.pub = rospy.Publisher('trajectory',Float32MultiArray, queue_size=10)
		rospy.Subscriber('map', OccupancyGrid, self.handleMap)
		rospy.Subscriber('slam_out_pose', PoseStamped, self.handlePose)
		rospy.Subscriber('target_pose', Float32MultiArray, self.handleTargetPose)
		self.map_width = 0
		self.map_height = 0
		self.random_range = 100
		self.dist_threshold = 7
		self.isolated_map = None
		self.found_path = False
		self.path_tree = []
		


	def convertXYtoIndex(self, x, y):
		indx = (self.map_width * y + x)/self.map_resolution
		return indx

	def handleMap(self, map_data):

		# getting map dimensions and resolution 
		if self.map_width == None and self.map_height == None:
			self.map_width = map_data.info.width
			self.map_width = map_data.info.height
			self.map_resolution = map_data.info.resolution
		# extracting only relevant map features
		if self.isolated_map == None:
			isolated_map = [(indx,x) for (indx,x) in enumerate(map_data.data) if x == 100 or x == 0]
			
		#rospy.loginfo(isolated_map)
		#rospy.loginfo(len(isolated_map))
	
	def handlePose(self, pose_data):
		pass
	
	def handleTargetPose(self, target_pose):
		pass

	def random_configuration(self):
		random_x = random.random()*self.random_range
		random_y = random.random()*self.random_range
		return (x,y)

	def get_euclidian_distance(point1, point2):
		distance = sqrt((point2[1] - point1[1])**2 + (point2[0] - point1[0])**2)
		return distance

	def nearest_vertex(self, random_point):
		distances = []
		for i in self.path_tree:
			x_i = i[1][0]
			y_i = i[1][1]
			new_distance = self.get_euclidian_distance((x_i,y_i), random_point)
			distances.append(new_distance)
		nearest_node = distances.index(min(distances))
		distances.clear()
		return nearest_node, distance
	
	def new_configuration(self, random_point, nearest_neighbor, distance):
		if distance > self.dist_threshold:
			self.getOtherNearestPoint(random_point, nearest_neighbor)
			return
		else:
			return random_point, nearest_neighbor

	def getOtherNearestPoint(self, random_point, nearest_neighbor):
		slope = (nearest_neighbor[1][1] - random_point[1])/(nearest_neighbor[1][0] - random_point[0])
		
		#initially set distance to infinity
		distance = float('inf')

		while distance > self.dist_threshold:
			# choosing random change factor with same sign as slope
			change = slope/abs(slope) * rand.random() 
			new_y = nearest_neighbor[1][1] - change
			new_x = nearest_neighbor[1][0] - (change/slope)
			distance = self.get_euclidian_distance(random_point, (new_x, new_y))
		
		return (new_x, new_y), nearest_neighbor
		
		

	def add_vertex(self, new_node):
		if len(self.path_tree) == 0:
			self.path_tree.append(new_node)

		else:	
			# adding new node to tree
			self.path_tree.append(new_node)
			# adding index of new_node to parent node
			self.path_tree[new_node[0]][-1].append(len(self.path_tree)-1)
		
		if get_euclidian_distance(new_node[1], self.destination_point) <=  self.dist_threshold:
			self.found_path = True
			final_node = [new_point, self.destination_point, []]
			self.path_tree.append(final_node)




	def doRRT(self):

		while not self.found_path:
			random_point = self.random_configuration()

			# making sure we only choose free points
			while self.grid_value(random_point) != 0:
				random_point = self.random_configuration()
			
			# get nearest node and ditance to it
			nearest_node, distance = self.nearest_vertex(random_point)
			
			# get new configuration point 
			new_point, nearest_node = self.new_configuration(random_point, nearest_neighbor, distance)
		
			new_node = [nearest_node, new_point, []]
			
			# adding the new point to path tree
			self.add_vertex(new_node)
			
			
			
			

	


if __name__ == '__main__':
	try:
		x = RRT()
		x.doRRT()
	except rospy.ROSInterruptException:
		rospy.loginfo('Rospy Exception Occured!')
