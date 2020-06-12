#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray

trajectory = None
grid = []
obstacle_indeces = []
obstacle_points = []
map_width = 2048
map_resolution = 0.05


def get_obstacle_points():
	global obstacle_indeces, obstacle_points, grid

	obstacle_indeces = [indx for (indx, x) in enumerate(grid) if x == 100]
	
	for i in obstacle_indeces:
		point = convertIndexToXY(i)
		obstacle_points.append(point)
	
def handle_grid(data):
	global grid 
	grid = data.data
				

def convertIndexToXY(index):
		w = map_width
		y = (-51.2 + (index/w)*map_resolution)
		x = (-51.2 + (index%w)*map_resolution)
		z = 0
		return (x,y,z)

def handleTrajectory(data):
	global trajectory
	if trajectory == None:
		trajectory = []
		temp =  data.data
		for i in range(0,len(temp),2):
			trajectory.append([temp[i], temp[i+1], 0])
	
	#rospy.loginfo('got trajectory %s', trajectory)



def visualize_path():
	global obstacle_indices, obstacle_points
	rospy.init_node('visualize_obstacles')
	#rospy.Subscriber('trajectory', Float32MultiArray, handleTrajectory)
	rospy.Subscriber('map', OccupancyGrid, handle_grid)
	pub = rospy.Publisher('obstacles', Marker, queue_size = 800)

	rospy.loginfo('getting indeces and points')

	while (len(obstacle_indeces) == 0) or (len(obstacle_points) == 0):
		get_obstacle_points()
		

	rospy.loginfo('gotem')

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# creating points marker instances
		points = Marker()
		

		# setting frame and stamp
		points.header.frame_id = "/map"
		points.header.stamp = rospy.Time.now()
		
		# setting namespace
		points.ns = "points_and_path"

		# setting type of action 
		points.action = points.ADD

		# setting w of pose
		points.pose.orientation.w = -1.0

		# setting position
		points.pose.position.x = 0.0
		points.pose.position.y = 0.0
		points.pose.position.z = 0.0

		# giving ids to markers
		points.id = 0

		# setting type
		points.type = points.POINTS

		# setting scale
		points.scale.x = 0.05
		points.scale.y = 0.05

		# points will be green
		points.color.g = 1.0
		points.color.a = 1.0


		for i in obstacle_points:
			curr_point = Point()
			curr_point.x = i[0]
			curr_point.y = i[1]
			curr_point.z = i[2]

			points.points.append(curr_point)

		pub.publish(points)

		rate.sleep()

	
if __name__ == '__main__':
	try:
		visualize_path()
	except rospy.ROSInterruptException:
		pass

