import rospy 
from std_srvs.srv import Trigger

class ArmCotrol(hm.HelloNode):
    """
    A simple encapsulation of the navigation stack for a Stretch robot.
    """
    def __init__(self):
        """
        Create an instance of the simple navigation interface.
        :param self: The self reference.
        """
        super().__init__()
        #For detection 
        self.switch_to_position_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_to_navigation_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
        self.joint_state = None 
        rospy.loginfo('{0}: Made contact with trajectory server'.format(self.__class__.__name__))
        

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
                #Add delta to joint_value
                new_value = joint_value + delta
                rospy.loginfo('To position %s', new_value)
            #Update location you wanna be at and move
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.trajectory_client.send_goal(trajectory_goal)
            self.trajectory_client.wait_for_result()

