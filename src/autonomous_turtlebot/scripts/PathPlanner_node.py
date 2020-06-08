#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped
from math import sqrt, degrees

class PathPlanner():
	
	def __init__(self):
		rospy.init_node('PathPlanner')
		self.pub = rospy.Publisher('reference_point', Float32MultiArray, queue_size=10)
		rospy.Subscriber('trajectory', Float32MultiArray, self.handle_trajectory)
		rospy.Subscriber('slam_out_pose', PoseStamped, self.handle_pose)
		self.trajectory = None
		self.curr_pose = None
		self.threshold = 0.09
		self.destination = Float32MultiArray()
		self.rate = rospy.Rate(10)
		self.initial_length = None

		# initializing multi array
		self.destination.layout.dim = [MultiArrayDimension()]
		self.destination.layout.dim[0].label = 'point'
		self.destination.layout.dim[0].size = 4
		self.destination.layout.dim[0].stride = 1
	

	def handle_trajectory(self, data):
	
		if self.trajectory == None:
			temp = list(data.data)
			trajectory = []
			for i in range(0, len(temp), 2):
				trajectory.append([temp[i], temp[i+1]])
			self.trajectory = trajectory
			self.trajectory.reverse()	
			self.initial_length = len(self.trajectory)			
			rospy.loginfo('got trajectory %s, len %s', self.trajectory, len(self.trajectory))


	def handle_pose(self, data):
		self.curr_pose = data.pose

	def plan_path(self):
		
		# wait for RRT node to finish
		while self.trajectory == None:
			pass

		# wait for current pose
		while self.curr_pose == None:
			pass

		#rospy.loginfo('starting path following')

		# while we still have points to reach, reach them
		while len(self.trajectory) != 0:
			curr_destination = self.trajectory.pop()
			self.destination.data = [curr_destination[0], curr_destination[1], 0,1]

			#rospy.loginfo('publishing curr point %s', curr_destination)
			rospy.loginfo('publishing node %s', self.initial_length - len(self.trajectory))
			rospy.loginfo('data %s', self.destination.data)
			# waiting for PID controller to get turtlebot close to point
			while self.not_arrived():
				#rospy.loginfo('distance %s', self.distance())
				self.pub.publish(self.destination)
				self.rate.sleep()
			rospy.loginfo('reached node')

		rospy.loginfo('goal destination reached')
		self.destination.data = [0,0,0,0]
		
		while not rospy.is_shutdown():
			self.pub.publish(self.destination)
			self.rate.sleep()

		
		
	
	def not_arrived(self):
		# getting curr orientation
		o = self.curr_pose.orientation
		curr_angles = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
		curr_yaw = curr_angles[-1]
		# getting curr distance from destination
		distance = self.distance()

		if (distance <= self.threshold) and (round(degrees(curr_yaw), 1) < 4.5) and (round(degrees(curr_yaw), 1) > -4.5):
			return False
		
		return True
		
	def distance(self):
		
		x = self.curr_pose.position.x
		y = self.curr_pose.position.y
		x2 = self.destination.data[0]
		y2 = self.destination.data[1]
		distance = sqrt(((y2-y)**2) + ((x2-x)**2))
		rospy.loginfo('distance %s', distance)
		
		return distance 
	

if __name__ == '__main__':
	try:
		x = PathPlanner()
		x.plan_path()

	except rospy.ROSSerializationException:
		rospy.loginfo('exception raised')


