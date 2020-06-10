#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from math import atan2, degrees, ceil, sqrt

desired_pose = None
map_spaces = None
got_point = None
curr_angular_diff = None
curr_angle = None
obstacles = None
map_width = 2048
half_width = map_width * .5
res = 0.05
origin = 51.2

def get_euclidian_distance(point1, point2):
		distance = sqrt((point2[1] - point1[1])**2 + (point2[0] - point1[0])**2)
		return distance

def get_point(data):
	distances = []
	hit = 0
	x = data.point.x
	y = data.point.y
	point = (x, y)
	point = (round(x, 3), round(y,3))
	
	#rospy.loginfo('clicked point: %s', point)
	#if point in obstacles:
		#rospy.loginfo('point is an obstacle')
	
	# checking distance to obstacle

	#rospy.loginfo('checking distance to obstacles')

	for i in obstacles:
		distance = get_euclidian_distance(i, point)
		#rospy.loginfo('obstacle %s, point %s, distance %s', i, point, distance)
		if distance < 0.2:
			hit+=1
	if hit > 0:
		rospy.loginfo('obstacle hit')
		hit = 0

	#rospy.loginfo('done checking distances')
			

def convertIndexToXY(index):
	
	w = map_width
	x = (origin - (index/w)*res)/2
	y = (origin - (index%w)*res)/2

	return(x,y)
	
	'''
	x = ((2*half_width - index)/map_width) * (res**2)
	y = (2*half_width - (x/(res**2))*map_width - index)*(res**2)
	'''
	'''
	x = (half_width - (index-half_width)/map_width) * (res**2)
	y = (res**2)*(half_width - index + (half_width-(x/res**2))*map_width)
	'''
	'''
	x = ( ceil(index/map_width)  - half_width) * -(.05**2)
	y = ((index - half_width) - map_width*(map_width*.5 - (x/(.05**2)))) * -(.05**2)
	'''


def get_grid(data):
	grid = data.data
	global obstacles, map_spaces
	temp = []
	obstacles = []

	temp = [indx for (indx,x) in enumerate(grid) if x == 100]
	map_spaces = [indx for (indx,x) in enumerate(grid) if x==100 or x ==0 ]

	rospy.loginfo('map spaces %s', map_spaces)
	rospy.loginfo('map space len %s', len(map_spaces))
	for i in temp:
		point = convertIndexToXY(i)
		obstacles.append(point)

	
def get_clicks():
	rospy.init_node('get_click_point')

	rospy.Subscriber('map', OccupancyGrid, get_grid)

	while obstacles == None:
		pass

	while len(obstacles) == 0:
		pass
		
	rospy.loginfo('got obstacle list, len %s', len(obstacles))

	#rospy.loginfo('obstacle %s', obstacles)

	rospy.Subscriber('clicked_point', PointStamped, get_point)
	
	(x,y) = convertIndexToXY(1024*2048 + 2048)
	
	index = (half_width - x/res)*map_width + half_width	

	#print('your point %s index %s', (x,y), index)


	while not rospy.is_shutdown():
		rospy.spin()		

if __name__ == '__main__':
	try:
		get_clicks()
	except rospy.ROSInterruptException:
		pass

