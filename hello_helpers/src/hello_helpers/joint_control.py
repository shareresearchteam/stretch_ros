import rospy 
from std_srvs.srv import Trigger
import hello_misc as hm 
# Import the FollowJointTrajectoryGoal from the control_msgs.msg package to
# control the Stretch robot
from control_msgs.msg import FollowJointTrajectoryGoal

# Import JointTrajectoryPoint from the trajectory_msgs package to define
# robot trajectories
from trajectory_msgs.msg import JointTrajectoryPoint
import math 
from std_msgs.msg import Float32
class JointControl(hm.HelloNode):
    """
    A simple encapsulation of the navigation stack for a Stretch robot.
    """
    def __init__(self):
        """
        Create an instance of the simple navigation interface.
        :param self: The self reference.
        """
        super().__init__()
        self.joint_state = None
        self.last_camera_angle = -math.pi/4
        rospy.loginfo('{0}: Made contact with trajectory server'.format(self.__class__.__name__))
        self.camera_angle_subscriber = rospy.Subscriber('Camera_Angle', Float32, self.camera_following_callback)

        

    def camera_following_callback(self, msg):
        rospy.loginfo("Received message: %s", msg)
        angle = -1*msg.data
        if abs(angle-self.last_camera_angle) > 0.1:
            rospy.loginfo('Current angle %s', self.last_camera_angle)
            rospy.loginfo('Changing camera angle from to %s radians', angle)
            new_pose = {'joint_head_tilt': angle}
            self.move_to_pose(new_pose)
            self.last_camera_angle = angle
        else: 
            rospy.loginfo('Angle not substantial for change in angle')


    def joint_states_callback(self, joint_state):
        '''
        Callback for the /stretch/joint_states topic to store the current joint states for use within the class
        '''
        self.joint_state = joint_state


    def send_command(self, command):
        """
        Handles single joint control commands by constructing a FollowJointTrajectoryMessage and sending it to
        trajectory_client in hello_misc
        """
        #Check current joint positions
        joint_state = self.joint_state
        #If joint info and command exists
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)

            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            #Based on set value
            if 'inc' in command:
                inc = command['inc']
                new_value = inc
            #Based on change in value 
            elif 'delta' in command:
                rospy.loginfo('Rotating %s', joint_name)
                
                #Check index and get value from position list
                joint_index = joint_state.name.index(joint_name)
                joint_value = joint_state.position[joint_index]
                delta = command['delta']
                #Add delta to joint_valuepose = {'joint_head_tilt': -pi/4, 'joint_head_pan': 0}
                new_value = joint_value + delta
                rospy.loginfo('To position %s', new_value)
            #Update location you wanna be at and move
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.trajectory_client.send_goal(trajectory_goal)
            self.trajectory_client.wait_for_result()
	
    def main(self):
        """
		Function that initiates the multipoint_command function.
		:param self: The self reference.
		"""
		# The arguments of the main function of the hm.HelloNode class are the
		# node_name, node topic namespace, and boolean (default value is true)
        hm.HelloNode.main(self, 'camera_follower', 'camera_follower', wait_for_first_pointcloud=False)
        pose = {'joint_head_tilt': -math.pi/4, 'joint_head_pan': 0}
        self.move_to_pose(pose)
        rospy.loginfo('completed intial setup...')
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    
    node = JointControl()
    node.main()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')