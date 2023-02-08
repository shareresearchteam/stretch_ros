#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32 
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
		self.rotate = 0
		self.current_state = None
		self.aruco_subscriber = rospy.Subscriber('ArUco_transform', TransformStamped, self.arucoTransformCallback)
		self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=10) #/stretch_diff_drive_controller/cmd_vel for gazebo
		self.current_state_subscriber = rospy.Subscriber('actions/current_state', StateMessage, self.current_state_callback)
		self.current_state_publisher = rospy.Publisher('actions/current_state', StateMessage, queue_size=10)
	
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
		if self.rotate >= 0.12:
			self.spin(negative=False)
		elif self.rotate <= -0.12:
			self.spin(negative=True)
		elif self.distance < -0.6:
			self.move_forward()
		else:
			rospy.loginfo("Close enough to target, updating state")
			new_state = None 
			if self.current_state != None: 
				new_state = State(self.current_state.name, 1)
				self.current_state_publisher.publish(new_state.toMsg())

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
	
	def arucoTransformCallback(self, msg:TransformStamped):
		"""
		Function that retrieves information about
		target aruco tag location. 
		"""
		self.distance = msg.transform.translation.x
		self.rotate = msg.transform.rotation.z
		rospy.loginfo("Comp dist %s", self.distance)
		rospy.loginfo("Comp rot %s", self.rotate)
		
	def main(self):
		while not rospy.is_shutdown():
			#self.spin(True)
			self.decide()

if __name__ == '__main__':
	try:
		rospy.init_node('movement')
		node = Move()
		node.main()
	except KeyboardInterrupt:
		rospy.loginfo("Keyboard Interrupt Shutting Down ")
