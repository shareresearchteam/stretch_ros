#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Twist, TransformStamped
from hello_helpers.msg import StateMessage
from state import State

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
		self.angle = 0
		self.current_state:State = State("",0,1)
		self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=10) #/stretch_diff_drive_controller/cmd_vel for gazebo
		self.current_state_subscriber = rospy.Subscriber('actions/current_state', StateMessage, self.current_state_callback)
		self.current_state_publisher = rospy.Publisher('actions/current_state', StateMessage, queue_size=10)
		self.flag_publisher = rospy.Publisher('actions/flag', Int8, queue_size=10)

		self.base_to_tag_angle_subscriber = rospy.Subscriber('base_to_tag_angle', Float32, self.base_to_tag_angle_callback)
		self.base_to_tag_distance_subscriber = rospy.Subscriber('base_to_tag_distance', Float32, self.base_to_tag_distance_callback)

	def base_to_tag_distance_callback(self,msg):
		self.distance = msg.data
	
	def base_to_tag_angle_callback(self, msg):
		self.angle = msg.data

	def current_state_callback(self, msg):
		self.current_state = State.fromMsg(msg) 

	def move_forward(self):
		"""
		Function that publishes Twist messages
		:param self: The self reference.

		:publishes command: Twist message.
		"""
		rospy.loginfo("Sending move command")
		command = Twist()
		command.linear.x = 0.1
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.0
		self.pub.publish(command)
	
	def spin(self, negative):
		""""
		Function that publishes Twist messages to spin Stretch
		"""
		rospy.loginfo("Sending rotate command")
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.05
		if negative:
			command.angular.z = -0.05
		self.pub.publish(command)
	
	def decide(self):
		#If it was a navigation signal that was received, move towards location
		#if self.last_transform_time > time.now() + 50:
		#	self.distance = 0
		#	self.rotate = 0
		if self.current_state.navigation: 
			if self.angle >= 0.12:
				self.flag_publisher.publish(0)
				self.spin(negative=False)
			elif self.angle <= -0.12:
				self.flag_publisher.publish(0)
				self.spin(negative=True)
			elif self.distance > 0.1:
				self.flag_publisher.publish(0)
				self.move_forward()
			else:
				self.flag_publisher.publish(1)
		else:
			rospy.loginfo("Command was not for navigation")

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
	
	def main(self):
		while not rospy.is_shutdown():
			self.decide() 
			


if __name__ == '__main__':
	try:
		rospy.init_node('movement')
		node = Move()
		node.main()
	except KeyboardInterrupt:
		rospy.loginfo("Keyboard Interrupt Shutting Down ")
