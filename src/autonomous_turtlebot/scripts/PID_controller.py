#!/usr/bin/env python

import rospy
import tf
from math import atan2, sqrt, degrees
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

class PIDController():
	def __init__(self):
		#initializing node, subscribers, and publishers
		rospy.init_node('PID_Controller')
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('reference_point', Float32MultiArray, self.get_goal_pose)
 		rospy.Subscriber('slam_out_pose', PoseStamped, self.get_curr_pose)
		# initializing PID parameters
		self.kp_v = .00010
		self.kd_v = .00001
		self.ki_v = .00009
		self.kp_w = .000013 
		self.kd_w = .000002
		self.ki_w = .000010
		# initializing position containers
		self.curr_pose = None
		self.goal_pose = None
		# initializing error terms
		self.e = None
		self.old_e =  0
		self.e_dot = 0
		self.e_int = 0
		# tolerance terms
		self.epsilon_w = 10
		self.epsilon_v = .18
		# velocity
		self.vel = Twist()
		# steering angle
		self.angle = 0

	# callback functions
	def get_curr_pose(self, data):
		self.curr_pose = data.pose
		o = self.curr_pose.orientation
		all_angles = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
   		self.angle = all_angles[-1] * 180/3.1415
		#rospy.loginfo("x: %s y: %s theta: %s", self.curr_pose.position.x, self.curr_pose.position.y, self.angle)
		
	def get_goal_pose(self, data):
		self.goal_pose = data.data

	# computes both types of angular error (straight line and towards distance)
	def get_angular_error(self, e_type):

		# gathering relevant orientation and coordinate info
		curr_crds = self.curr_pose.position
		o = self.curr_pose.orientation
		curr_angle = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
		goal_pose = self.goal_pose

		rospy.loginfo('type %s curr_angle %s', e_type, curr_angle[-1])

		# if this is the first step
		if e_type == 1:
			a_diff = atan2((goal_pose[1] - curr_crds.y),(goal_pose[0] - curr_crds.x))
			#rospy.loginfo("curr_pos %s %s, goal %s %s", curr_crds.x, curr_crds.y, goal_pose[0], goal_pose[1])
			#rospy.loginfo("a_diff %s, curr_angle %s, 'error' %s", degrees(a_diff), degrees(curr_angle[-1]), degrees(a_diff - curr_angle[-1]))
			return degrees(a_diff - curr_angle[-1])

		# if this is the last step		
		if e_type == 2:
			return degrees(goal_pose[2] - curr_angle[-1])
		
	
	def get_euclidian_error(self):
		curr = self.curr_pose.position
		error = sqrt( (self.goal_pose[1] - curr.y)**2 + (self.goal_pose[0] - curr.x)**2)
		return error

	def PID(self, error, vel_type):
		e_dot = error - self.old_e
		self.e_int = self.e_int + error
		if vel_type == 'angular':		
			self.vel.angular.z = self.kp_w*error + self.kd_w*e_dot + self.ki_w*self.e_int
		if vel_type == 'linear':
			self.vel.linear.x = self.kp_v*error + self.kd_v*e_dot + self.ki_v*self.e_int

		self.pub.publish(self.vel)
		self.old_e = error

	def move_PID(self, movement_type, step):
		error_thresh = 0
		while True:
			if movement_type == 'angular':
				error = self.get_angular_error(step)
				error_thresh = self.epsilon_w
			if movement_type == 'linear':
				error = self.get_euclidian_error()
				error_thresh = self.epsilon_v
		
			self.PID(error, movement_type)
			
			#rospy.loginfo("error =%s type=%s curr_angle= %s x= %s y =%s goalx %s goaly %s goaltheta %s", error, step, self.angle, self.curr_pose.position.x, self.curr_pose.position.y, self.goal_pose[0], self.goal_pose[1], self.goal_pose[2])
			if abs(error) <= error_thresh:
				self.e_int = 0
				self.e_dot = 0
				self.old_e = 0
				self.vel.linear.x = 0
				self.vel.linear.y = 0
				self.vel.angular.z = 0
				self.pub.publish(self.vel)
				break
			
	def method1(self):
		self.move_PID('angular', 1)
		self.move_PID('linear' , 0)
		self.move_PID('angular', 2)
		
			
	def method2(self):
		pass

				
	# main go to goal function
	def go_to_goal(self):

		# we wait until we have curr_pose and goal_pose before beginning
		while self.goal_pose == None or self.curr_pose == None:
			pass

		previous_pose = self.goal_pose
		
		while self.goal_pose != None:
			if self.goal_pose[3]== 1:
				self.method1()
			elif self.goal_pose[3] == 2:
				self.method2()
			
			# waiting for new pose
			while self.goal_pose == previous_pose:
				pass 
			
			rospy.loginfo('got new pose %s', self.goal_pose)
			
						
if __name__ == '__main__':
	try:
		x = PIDController()
		x.go_to_goal()
	except rospy.ROSInterruptException:
		rospy.loginfo('Exception Raised!')


			
		
