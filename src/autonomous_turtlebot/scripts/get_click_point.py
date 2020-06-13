#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from math import atan2, degrees, ceil, sqrt

desired_pose = None
got_point = None
curr_angular_diff = None
curr_angle = None
obstacles = None
map_width = 2048
res = 0.05

def get_euclidian_distance(point1, point2):
		distance = sqrt((point2[1] - point1[1])**2 + (point2[0] - point1[0])**2)
		return distance

def get_point(data):
	distances = []
	x = data.point.x
	y = data.point.y
	point = (x, y)
	point = (round(x, 3), round(y,3))
	
	rospy.loginfo('clicked point: %s', point)
	if point in obstacles:
		rospy.loginfo('point is an obstacle')
	
	# checking distance to obstacle

	rospy.loginfo('checking distance to obstacles')

	for i in obstacles:
		distance = get_euclidian_distance(i, point)
		#rospy.loginfo('obstacle %s, point %s, distance %s', i, point, distance)
		if distance < 0.1:
			rospy.loginfo('obstacle threshold hit')

	rospy.loginfo('done checking distances')

def convertIndexToXY(index):

	half_width = map_width * .5
	x = (half_width - (index/map_width)) *res
	y = res*(2*half_width - index - (x/res))
	return (x,y)


def get_grid(data):
	grid = data.data
	global obstacles
	temp = []
	obstacles = []

	temp = [indx for (indx,x) in enumerate(grid) if x == 100]
	for i in temp:
		point = convertIndexToXY(i)
		obstacles.append(point)

	
def get_pose(data):
	curr_pose = data.pose
	curr_crds = curr_pose.position
	o = curr_pose.orientation
	all_angles = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
	curr_angle = all_angles[-1]

	a_diff = atan2((desired_pose.data[1] - curr_crds.x),(desired_pose.data[0] - curr_crds.y))
	#rospy.loginfo('degrees %s', degrees(a_diff - curr_angle))
	
def get_clicks():
	rospy.init_node('get_click_point')

	rospy.Subscriber('map', OccupancyGrid, get_grid)

	while obstacles == None:
		pass

	while len(obstacles) == 0:
		pass
		
	rospy.loginfo('got obstacle list, len %s', len(obstacles))

	rospy.Subscriber('clicked_point', PointStamped, get_point)

	while not rospy.is_shutdown():
		rospy.spin()		

if __name__ == '__main__':
	try:
		get_clicks()
	except rospy.ROSInterruptException:
		pass

