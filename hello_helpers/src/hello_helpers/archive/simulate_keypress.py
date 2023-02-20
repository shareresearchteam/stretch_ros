#!/usr/bin/env python3
# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot
import rospy
from control_msgs.msg import FollowJointTrajectoryGoal


# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint

#pose = {'gripper_aperture': 0.125}
#                self.move_to_pose(pose)

# Import hello_misc script for handling trajectory goals with an action client
import hello_helpers.hello_misc as hm

class MultiPointCommand(hm.HelloNode):
	"""
	A class that sends multiple joint trajectory goals to the stretch robot.
	"""

	def __init__(self, shape):
		"""
		Function that initializes the inhereted hm.HelloNode class.
		:param self: The self reference.
		"""
		hm.HelloNode.__init__(self)
		converted_shape = []
		for point in shape:
			trajectory = JointTrajectoryPoint()
			trajectory.positions = [point[0], point[1],point[2], point[3], point[4]]
			converted_shape.append(trajectory)
		self.path = converted_shape

	
	def createPath(self, shape):
		converted_shape = []
		for point in shape:
			trajectory = JointTrajectoryPoint()
			trajectory.positions = [point[0], point[1],point[2], point[3], point[4]]
			converted_shape.append(trajectory)
		return converted_shape

	def issue_multipoint_command(self, shape):
		"""
		Function that makes an action call and sends multiple joint trajectory goals
		to the joint_lift, wrist_extension, and joint_wrist_yaw.
		:param self: The self reference.
		"""

		# Set trajectory_goal as a FollowJointTrajectoryGoal and define
		# the joint names as a list

		trajectory_goal = FollowJointTrajectoryGoal()
		trajectory_goal.trajectory.joint_names = ['wrist_extension','joint_lift', 'joint_wrist_yaw','joint_wrist_pitch']

		# Then trajectory_goal.trajectory.points is defined by a list of the joint
		# trajectory points
		trajectory_goal.trajectory.points = self.createPath()

		# Specify the coordinate frame that we want (base_link) and set the time to be now
		trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
		trajectory_goal.trajectory.header.frame_id = 'base_link'

		# Make the action call and send the goal. The last line of code waits
		# for the result before it exits the python script
		self.trajectory_client.send_goal(trajectory_goal)
		rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
		self.trajectory_client.wait_for_result()

	def main(self):
		"""
		Function that initiates the multipoint_command function.
		:param self: The self reference.
		"""
		# The arguments of the main function of the hm.HelloNode class are the
		# node_name, node topic namespace, and boolean (default value is true)
		hm.HelloNode.main(self, 'multipoint_command', 'multipoint_command', wait_for_first_pointcloud=False)
		rospy.loginfo('issuing multipoint command...')
		self.issue_multipoint_command()
		rospy.sleep(2)

if __name__ == '__main__':
	pi = 3.14
	length = 0.06
	try:
        # Instanstiate a `MultiPointCommand()` object and execute the main() method
		pointlist = [[0,0.5,-0.4,0,0],[0,0.5,-0.4,0,0.4],[0,0.5,-0.4,0,-0.4],[0,0.5,-0.4,0,0],[0,0.5,-0.4,-pi/4,0],[0,0.5,-0.4,-pi/4,0],[0,0.8,-0.4,-pi/4,0],[length,0.8,2.2,-pi/4,0],[length,0.7,2.2,-pi/4,0],[length,0.8,2.2,-pi/4,0], [0,0.8,-0.4,0,0], [0,0.5,-0.4,0,0]]
		node = MultiPointCommand(shape=pointlist)
		node.main()	

		length = 0.08
		pointlist = [[0,0.5,-0.4,0,0],[0,0.5,-0.4,0,0.4],[0,0.5,-0.4,0,-0.4],[0,0.5,-0.4,0,0],[0,0.5,-0.4,-pi/4,0],[0,0.5,-0.4,-pi/4,0],[0,0.8,-0.4,-pi/4,0],[length,0.8,2.2,-pi/4,0],[length,0.7,2.2,-pi/4,0],[length,0.8,2.2,-pi/4,0], [0,0.8,-0.4,0,0], [0,0.5,-0.4,0,0]]
		node.new_list(pointlist)
		node.issue_multipoint_command()
		rospy.sleep(2)
		length = 0.04
		pointlist = [[0,0.5,-0.4,0,0],[0,0.5,-0.4,0,0.4],[0,0.5,-0.4,0,-0.4],[0,0.5,-0.4,0,0],[0,0.5,-0.4,-pi/4,0],[0,0.5,-0.4,-pi/4,0],[0,0.8,-0.4,-pi/4,0],[length,0.8,2.2,-pi/4,0],[length,0.7,2.2,-pi/4,0],[length,0.8,2.2,-pi/4,0], [0,0.8,-0.4,0,0], [0,0.5,-0.4,0,0]]
		node.new_list(pointlist)
		node.issue_multipoint_command()
		rospy.sleep(2)
		length = 0.1
		pointlist = [[0,0.5,-0.4,0,0],[0,0.5,-0.4,0,0.4],[0,0.5,-0.4,0,-0.4],[0,0.5,-0.4,0,0],[0,0.5,-0.4,-pi/4,0],[0,0.5,-0.4,-pi/4,0],[0,0.8,-0.4,-pi/4,0],[length,0.8,2.2,-pi/4,0],[length,0.7,2.2,-pi/4,0],[length,0.8,2.2,-pi/4,0], [0,0.8,-0.4,0,0], [0,0.5,-0.4,0,0]]
		node.new_list(pointlist)
		node.issue_multipoint_command()
		rospy.sleep(2)
		length = 0.02
		pointlist = [[0,0.5,-0.4,0,0],[0,0.5,-0.4,0,0.4],[0,0.5,-0.4,0,-0.4],[0,0.5,-0.4,0,0],[0,0.5,-0.4,-pi/4,0],[0,0.5,-0.4,-pi/4,0],[0,0.8,-0.4,-pi/4,0],[length,0.8,2.2,-pi/4,0],[length,0.7,2.2,-pi/4,0],[length,0.8,2.2,-pi/4,0], [0,0.8,-0.4,0,0], [0,0.5,-0.4,0,0]]
		node.new_list(pointlist)
		node.issue_multipoint_command()
		rospy.sleep(2)

	except KeyboardInterrupt:
		rospy.loginfo('interrupt received, so shutting down')
