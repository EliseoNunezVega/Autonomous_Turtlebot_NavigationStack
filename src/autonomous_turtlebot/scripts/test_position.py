#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def transmit_desired_pose():
	rospy.init_node('ref_publisher')
	pub = rospy.Publisher('target_pose', Float32MultiArray, queue_size = 10)
	rate = rospy.Rate(10)
	desired_pose = Float32MultiArray()

	x = 0.9
	y = 0.9
	theta = 0.0 
	mode = 1
	
	desired_pose.data.append(x)
	desired_pose.data.append(y)
	desired_pose.data.append(theta)
	desired_pose.data.append(mode)
	
	while not rospy.is_shutdown():
		pub.publish(desired_pose)
		rate.sleep()

if __name__ == '__main__':
	try:
		transmit_desired_pose()
	except rospy.ROSInterruptException:
		pass

