#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from math import atan2, degrees

desired_pose = None
got_point = None
curr_angular_diff = None
curr_angle = None


def get_point(data):
	global got_point
	global desired_pose
	desired_pose = Float32MultiArray()
	got_point = True
	x = data.point.x
	y = data.point.y
	desired_pose.data.append(x)
	desired_pose.data.append(y)
	desired_pose.data.append(0)
	desired_pose.data.append(1)
	rospy.loginfo(desired_pose)
	
def get_pose(data):
	curr_pose = data.pose
	curr_crds = curr_pose.position
	o = curr_pose.orientation
	all_angles = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
	curr_angle = all_angles[-1]

	a_diff = atan2((desired_pose.data[1] - curr_crds.x),(desired_pose.data[0] - curr_crds.y))
	rospy.loginfo('degrees %s', degrees(a_diff - curr_angle))
	
def get_clicks():
	rospy.init_node('get_click_point')
	rospy.Subscriber('clicked_point', PointStamped, get_point)

	while desired_pose == None:
		pass

	rospy.Subscriber('slam_out_pose', PoseStamped, get_pose)

	pub = rospy.Publisher('reference_point', Float32MultiArray, queue_size = 10)
	rate = rospy.Rate(10)
	
	while not got_point:
		pass
	
	while not rospy.is_shutdown():
		pub.publish(desired_pose)
		rate.sleep()		

if __name__ == '__main__':
	try:
		get_clicks()
	except rospy.ROSInterruptException:
		pass

