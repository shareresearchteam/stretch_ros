#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TransformStamped

class Move:
	"""
	A class that sends Twist messages to move the Stretch robot forward.
	"""
	def __init__(self):
		"""
		Function that initializes the subscriber.
		:param self: The self reference.
		"""
		self.temp_bool = False
		self.aruco_subscriber = rospy.Subscriber('ArUco_transform', TransformStamped, self.arucoTransformCallback)
		self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1) #/stretch_diff_drive_controller/cmd_vel for gazebo

	def move_forward(self):
		"""
		Function that publishes Twist messages
		:param self: The self reference.

		:publishes command: Twist message.
		"""
		command = Twist()
		command.linear.x = 0.3
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0
		self.pub.publish(command)
	
	def moveUpCondition(self, distance):
		rospy.loginfo(self.temp_bool)
		if not self.temp_bool:
			self.move_forward()
			time = distance/0.3
			start_time = rospy.Time.now()
			while rospy.Time.now() < start_time + rospy.Duration.from_sec(time):
			    self.move_forward()
			    time.sleep(.25)
			self.stop()
			self.temp_bool = True
		    
	
	def stop(self):
		"""
		Function that stops Stretch
		"""
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0
		self.pub.publish(command)
	
	def arucoTransformCallback(self, msg):
		"""
		Function that retrieves information about
		target aruco tag location.
		Returns  
		"""
		rospy.loginfo("Received this info %s", msg)
		self.moveUpCondition(msg.transform.translation.x)
	
	

if __name__ == '__main__':
	rospy.init_node('move')
	base_motion = Move()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rospy.spin()
