#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

def handlePose(data):
	pose = data.pose.position
	rospy.loginfo('x: %s, y: %s', pose.x, pose.y)

def transmit_desired_pose():
	rospy.init_node('ref_publisher')
	rospy.Subscriber('slam_out_pose', PoseStamped,handlePose)


	while not rospy.is_shutdown():
		rospy.spin()



if __name__ == '__main__':
	try:
		transmit_desired_pose()
	except rospy.ROSInterruptException:
		pass

