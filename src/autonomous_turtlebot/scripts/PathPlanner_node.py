#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import PoseStamped
from math import sqrt

class PathPlanner():
	
	def __init__(self):
		rospy.init_node('PathPlanner')
		self.pub = rospy.Publisher('reference_point', Float32, queue_size=10)
		rospy.Subscriber('trajectory', Float32MultiArray, self.handle_trajectory)
		rospy.Subscriber('slam_out_pose', PoseStamped, self.handle_pose)
		self.trajectory = None
		self.curr_pose = None
		self.treshold = 0.1
		self.destination = Float32()


	def handle_trajectory(self, data):
		if self.trajectory == None:
			temp = list(data.data)
			trajectory = []
			for i in range(0, len(temp), 2):
				trajectory.append([temp[i], temp[i+1]])
			self.trajectory = trajectory
			self.trajectory.reverse()				
			rospy.loginfo('got trajectory %s', self.trajectory)



	def handle_pose(self, data):
		self.curr_pose = data.pose
		pass

	def plan_path(self):
		
		# wait for RRT node to finish
		while self.trajectory == None:
			pass

		# wait for current pose
		while self.curr_pose == None:
			pass

		# while we still have points to reach, reach them
		while len(self.trajectory) != 0:
			curr_destination = self.trajectory.pop(0)
			self.destination.data = [curr_destination[0], curr_destination[1], 0]
			self.pub.publish(self.destination)
		
			while self.distance < self.treshold:
				pass
		
	def distance(self):
		
		x = self.curr_pose.position.x
		y = self.curr_pose.position.y
		x2 = self.destination[0]
		y2 = self.destination[1]

		return sqrt(((y2-y)**2) + ((x2-x)**2))
	

if __name__ == '__main__':
	try:
		x = PathPlanner()
		x.plan_path()

	except rospy.ROSException:
		rospy.loginfo('exception raised')


