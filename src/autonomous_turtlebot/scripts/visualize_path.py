#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray

trajectory = None


def handleTrajectory(data):
	global trajectory
	if trajectory == None:
		trajectory = []
		temp =  data.data
		for i in range(0,len(temp),2):
			trajectory.append([temp[i], temp[i+1], 0])
	
	#rospy.loginfo('got trajectory %s', trajectory)



def visualize_path():
	rospy.init_node('visualize_path')
	rospy.Subscriber('trajectory', Float32MultiArray, handleTrajectory)
	pub = rospy.Publisher('robot_path', Marker, queue_size = 30)

	# waiting to obtain trajectory
	while trajectory == None:
		pass

	
	rate = rospy.Rate(20)

	while not rospy.is_shutdown():
		# creating path and points marker instances
		points = Marker()
		path = Marker()

		# setting frame and stamp
		points.header.frame_id = path.header.frame_id = "/map"
		points.header.stamp = path.header.stamp = rospy.Time.now()
		
		# setting namespace
		points.ns = path.ns = "points_and_path"

		# setting type of action 
		points.action = points.ADD
		path.action = path.ADD

		# setting w of pose
		points.pose.orientation.w = path.pose.orientation.w = 1.0

		# setting position
		points.pose.position.x = path.pose.position.x = 0.0
		points.pose.position.y = path.pose.position.y = 0.0
		points.pose.position.z = path.pose.position.z = 0.0

		# giving ids to markers
		points.id = 0
		path.id = 1

		# setting type
		points.type = points.POINTS
		path.type = path.LINE_STRIP

		# setting scale
		points.scale.x = 0.05
		points.scale.y = 0.05

		path.scale.x = 0.05

		# points will be green
		points.color.g = 1.0
		points.color.a = 1.0

		# path will be blue
		path.color.b = 1.0
		path.color.a = 1.0

		for i in trajectory:
			curr_point = Point()
			curr_point.x = i[0]
			curr_point.y = i[1]
			curr_point.z = i[2]

			points.points.append(curr_point)
			path.points.append(curr_point)

		pub.publish(points)
		pub.publish(path)

		rate.sleep()

	
if __name__ == '__main__':
	try:
		visualize_path()
	except rospy.ROSInterruptException:
		pass

