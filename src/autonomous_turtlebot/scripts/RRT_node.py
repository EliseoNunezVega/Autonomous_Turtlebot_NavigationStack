#!/usr/bin/env python

import rospy
import random 
from math import sqrt, ceil 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class RRT():
	def __init__(self):
		rospy.init_node('RRT_node')
		self.pub = rospy.Publisher('trajectory',Float32MultiArray, queue_size=20)
		rospy.Subscriber('map', OccupancyGrid, self.handleMap)
		rospy.Subscriber('slam_out_pose', PoseStamped, self.handlePose)
		rospy.Subscriber('target_pose', Float32MultiArray, self.handleTargetPose)
		self.map_width = None
		self.map_height = None
		self.obstacle_indices = None
		self.obstacle_points = []
		self.map_resolution = None
		self.previous_indices = []
		self.random_range = 2.5
		self.dist_threshold = .5
		self.destination_point = None
		self.initial_point = None
		self.free_spaces_indices = None
		self.max_iterations = 5000
		self.occupancy_grid = None
		self.found_path = False
		self.path_tree = []
		self.final_path = []
		self.trajectory = Float32MultiArray()
		self.rate = rospy.Rate(10)

		


	def convertXYtoIndex(self, x, y):
		indx = self.map_width* (self.map_width * .5 - ceil(x/(self.map_resolution**1))) + (self.map_width * .5 - ceil(y/(self.map_resolution**1)))
		return int(indx)

	def convertIndexToXY(self, index):
		#index = index * (.05 ** 3)
		x = (int(index/self.map_width)  - self.map_width*.5) * -(.05**1)
		y = ((index - self.map_width*.5) - self.map_width*(self.map_width*.5 - (x/(.05**1)))) * -(.05**1)
		return (x,y)

	def get_free_and_obstacle_indices(self):
		# extracting only relevant unblocked points
		grid = self.occupancy_grid
		self.free_spaces_indices = [indx for (indx,x) in enumerate(grid) if x == 0]
	
		# extrcating obstacle points for later use
		grid = self.occupancy_grid
		self.obstacle_indices = [indx for (indx,x) in enumerate(grid) if x == 100]
		for i in self.obstacle_indices:
			point = self.convertIndexToXY(i)
			self.obstacle_points.append(point)

	def handleMap(self, map_data):
		
		# getting occupancy data
		self.occupancy_grid = map_data.data
		
		# getting map dimensions and resolution 
		if self.map_width == None or self.map_height == None or self.map_resolution == None:
			self.map_width = map_data.info.width
			self.map_width = map_data.info.height
			self.map_resolution = map_data.info.resolution

		if (self.free_spaces_indices == None) or (self.obstacle_indices == None):
			self.get_free_and_obstacle_indices()
			
			
		#rospy.loginfo(isolated_map)
		#rospy.loginfo(len(isolated_map))
	
	def handlePose(self, pose_data):
		if self.initial_point == None:
			x = pose_data.pose.position.x
			y = pose_data.pose.position.y

			self.initial_point = (x,y)
		
	
	def handleTargetPose(self, target_pose):
		if self.destination_point == None:
			self.destination_point = (target_pose.data[0], target_pose.data[1])
	
	def point_hits_obstacle(self, point):
		
		for obstacle in self.obstacle_points:
			distance = self.get_euclidian_distance(point, obstacle)
			if distance < 0.4 :
				#rospy.loginfo('distance %s', distance)
				return True
	
		return False


	def random_configuration(self):
		random_index = random.choice(self.free_spaces_indices)
		random_point = self.convertIndexToXY(random_index)
		random_x = random_point[0]
		random_y = random_point[1]
		#rospy.loginfo('random index %s', random_index)
		#rospy.loginfo('random point (%s, %s)', random_x, random_y)

		return (random_x,random_y)

	def get_euclidian_distance(self, point1, point2):
		distance = sqrt((point2[1] - point1[1])**2 + (point2[0] - point1[0])**2)
		return distance

	def nearest_vertex(self, random_point):
		distances = []
		nearest_distance = None

		for i in self.path_tree:
			x_i = i[1][0]
			y_i = i[1][1]
			new_distance = self.get_euclidian_distance((x_i,y_i), random_point)
			distances.append(new_distance)

		nearest_node_indx = distances.index(min(distances))
		#nearest_node = self.path_tree[nearest_node_indx]
		nearest_distance = min(distances)

		return nearest_node_indx, nearest_distance
	
	def new_configuration(self, random_point, nearest_neighbor, distance):
			
		if distance > self.dist_threshold:
			#rospy.loginfo('point not close enough, choosing other point')
			new_point = self.getOtherNearestPoint(random_point, nearest_neighbor)
			return new_point, nearest_neighbor
		else:
			return random_point, nearest_neighbor

	def getOtherNearestPoint(self, random_point, nearest_neighbor):
		
		distance = float('inf')
		nearest_point = self.path_tree[nearest_neighbor][1]
		temp_free_spaces = self.free_spaces_indices
		new_random_point = random_point

		

		while (distance > self.dist_threshold) or (self.point_hits_obstacle(new_random_point)):		
			random_index = random.choice(temp_free_spaces)
			temp_free_spaces.remove(random_index)
			new_random_point = self.convertIndexToXY(random_index)
			distance = self.get_euclidian_distance(new_random_point, nearest_point)
			
		return new_random_point
		

	def correct_same_y(self, neighbor_point, random_point):
		distance = float('inf')
		sign = random_point[1] - neighbor_point[1] / abs(random_point[1] - neighbor_point[1])
		change = 0

		while distance > self.dist_threshold:
			new_y += sign * random.uniform(0, .1)
			distance = self.get_euclidian_distance(random_point, (neighboring_point[0], new_y))
		return (neighbor_point[0], new_y)

	def correct_same_x(self, neighbor_point, random_point):
		
		distance = float('inf')
		sign = random_point[0] - neighbor_point[0] / abs(random_point[0] - neighbor_point[0])
		change = 0

		while distance > self.dist_threshold:
			new_x += sign * random.uniform(0, .1)
			distance = self.get_euclidian_distance(random_point, (new_x, neighbor_point[1]))

		return (new_x, neighbor_point[1])	
			
	def correct_sloped_points(self, neighbor_point, random_point, dy, dx):
		slope = dy/dx
		sign_slope = slope/abs(slope)
		sign_y = dy/abs(dy)
		sign_x = dx/abs(dx)

		#rospy.loginfo('sign x %s sign y %s', sign_x, sign_y)
		#rospy.loginfo('random point %s', random_point)		
		#rospy.loginfo('first distance %s', self.get_euclidian_distance(random_point, neighbor_point))
		
		#initially set distance to infinity
		distance = float('inf')
		# choosing random change factor with same sign as slope
		change = 0
		change_y = 0
		change_x = 0

		while distance > self.dist_threshold:
			#rospy.loginfo('looking for new point')
			change += random.uniform(0,.1)
			change_y = change * sign_y
			change_x = (change/abs(slope))*  sign_x
			new_y = random_point[1] - change_y
			new_x = random_point[0] - change_x
			distance = self.get_euclidian_distance(random_point, (new_x, new_y))
			#rospy.loginfo('new distance %s', distance)
		
		#rospy.loginfo('done getting point slope')
		return (new_x, new_y)
	
	

	def add_vertex(self, new_node):
		if len(self.path_tree) == 0:
			self.path_tree.append(new_node)

		else:	
			#rospy.loginfo('new_node %s', new_node)
			# adding new node to tree
			self.path_tree.append(new_node)
			# adding index of new_node to parent node
			self.path_tree[new_node[0]][-1].append(len(self.path_tree)-1)
		
		# checking if new point is close enough to final destination point
		new_point = new_node[1]
		destination_point = self.destination_point
		#rospy.loginfo('new point %s, destination_point %s', new_point, destination_point)

		distance_to_target = self.get_euclidian_distance(new_point, destination_point) 
		if  distance_to_target <=  self.dist_threshold:
			self.found_path = True
			final_node = [self.path_tree.index(new_node), self.destination_point, []]
			self.path_tree.append(final_node)


	def grid_value(self,point):
		half_width = (self.map_width/2)
		scaled_y = ceil(point[1]/(self.map_resolution**2))
		scaled_x = ceil(point[0]/(self.map_resolution**2))
		index = half_width*2*(-scaled_x + half_width) + (-scaled_y + half_width)
		#rospy.loginfo('index %s', index)		
		return self.occupancy_grid[int(index)]

	def retrace_path(self):
		final_path = []
		parent = self.path_tree[-1]

		while parent[0] != None:
			#rospy.loginfo('parent %s', parent)
			final_path.append(parent[1])
			parent = self.path_tree[parent[0]]
		
		return final_path


	def doRRT(self):

		
		
		# waiting for destination point and grid data
		while (self.destination_point == None) or (self.occupancy_grid == None) or (self.map_resolution == None) or (self.free_spaces_indices == None):
			pass
		rospy.loginfo('got destination point')

		while self.free_spaces_indices == None:
			pass	
		rospy.loginfo('got free spaces array')

		while self.obstacle_indices == None:
			pass
		rospy.loginfo('got obstacles array')

		while self.initial_point == None:
			pass

		rospy.loginfo('got curr position')

		while (len(self.obstacle_indices) == 0) or (len(self.free_spaces_indices)==0):
			self.get_free_and_obstacle_indices()
			pass

	
		rospy.loginfo('len of occupancy grid %s', len(self.occupancy_grid))
		rospy.loginfo('len of free spaces array %s', len(self.free_spaces_indices))
		rospy.loginfo('len of obstacles array %s', len(self.obstacle_indices))

	
		# appending our start node with initial position
		start_node = [None, self.initial_point, []]
	
		self.path_tree.append(start_node)

		iteration = 0
		
		rospy.loginfo('Starting Process')

		while not self.found_path and iteration < self.max_iterations:

			#rospy.loginfo('getting random point')

			# getting random point configuration
			random_point = self.random_configuration()

			#rospy.loginfo('got random point')
			
			#rospy.loginfo('staring with nearest_vertex')

			# get nearest node and distance to it
			nearest_node, distance = self.nearest_vertex(random_point)
			
			#rospy.loginfo('done with nearest vertex')
		
			#rospy.loginfo('starting new configuration')
	
			# get new configuration point 
			new_point, nearest_node = self.new_configuration(random_point, nearest_node, distance)

			#rospy.loginfo('done with new configuration')

			#rospy.loginfo('starting add vertex')

			new_node = [nearest_node, new_point, []]
			
			# adding the new point to path tree
			self.add_vertex(new_node)

			#rospy.loginfo('done with add_vertex')
			
			iteration += 1
			
			#rospy.loginfo('this is the path tree so far %s', self.path_tree)

		rospy.loginfo('path found and complete')
		#rospy.loginfo('path tree %s', self.path_tree)		
		self.final_path = self.retrace_path()
		self.final_path.reverse()
		#rospy.loginfo(self.final_path)
		
		# formatting trajectory MultiArray
		self.trajectory.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
	
		self.trajectory.layout.dim[0].label = "point"
		self.trajectory.layout.dim[0].size = len(self.final_path)
		self.trajectory.layout.dim[0].stride = 2

		self.trajectory.layout.dim[1].label = "xyvalue"
		self.trajectory.layout.dim[1].size = 2
		self.trajectory.layout.dim[1].stride = 1

		# flattening our data 
		flattened_path = []
		for point in self.final_path:
			flattened_path.append(point[0])
			flattened_path.append(point[1])	

		# attaching our data to the trajectory msg
		self.trajectory.data = flattened_path

		rospy.loginfo('trajectory data %s', self.trajectory.data)	

		while not rospy.is_shutdown():
			self.pub.publish(self.trajectory)
			self.rate.sleep()			


if __name__ == '__main__':
	try:
		x = RRT()
		x.doRRT()
	except rospy.ROSInterruptException or rospy.ROSException:
		rospy.loginfo('path %s', x.path_tree)
		rospy.loginfo('Rospy Exception Occured!')
