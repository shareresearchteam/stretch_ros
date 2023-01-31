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
		self.distance = 0
		self.rotation = 0
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
	
	def decide(self):
		if self.distance < -0.35:
			self.move_forward()

	def moveUpCondition(self):
		rospy.loginfo(self.temp_bool)
		while self.distance > 0.1:
			self.move_forward()
			rospy.sleep()
		    
	
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
		self.distance = msg.transform.translation.x
		self.rotate = msg.transform.rotation.z
		rospy.loginfo(self.distance)
		
	def main(self):
		while not rospy.is_shutdown():
			self.move_forward()

if __name__ == '__main__':
	try:
		rospy.init_node('movement')
		node = Move()
		node.main()
	except KeyboardInterrupt:
		rospy.loginfo("Keyboard Interrupt Shutting Down ")