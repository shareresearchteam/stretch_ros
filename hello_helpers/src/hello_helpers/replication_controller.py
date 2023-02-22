#Remember to chmod and add to the CMake? 
import hello_misc as hm

import rospy
import tf2_ros
from math import pi, sqrt, atan2

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import JointState
import math 
from std_msgs.msg import Float32, Int8
import numpy as np 
from hello_helpers.msg import StateMessage
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
from state import State 
from base_control import Move
import stretch_funmap.navigate as nv

class ReplicationController(hm.HelloNode):
    def __init__(self):

        super().__init__()
        
        self.joint_state = None
        self.move_base = nv.MoveBase(self)
        #Controls how many duplicates it will tolerate before defaulting
        self.count = 0

        #Initial data for camera movement
        self.last_camera_angle = -math.pi/4
        self.last_pan_camera_angle = 0
        self.cam_to_tag_angle = -math.pi/4
        self.base_to_tag_angle = 0
        self.base_to_tag_distance = 0

        min_pan = -4
        max_pan = 1.3
        self.delta = (max_pan - min_pan) / 10
        self.search_flag = True
        #For state control
        self.current_state = "nav_1"
        self.current_state_index = 0
        self.states = ["nav_1","nav_2","table","nav_3","nav_5"]
        
        self.last_transform = Transform()
    
        #For controlling base
        self.base_commands = Move()

    def joint_states_callback(self, joint_state):
        '''
        Callback for the /stretch/joint_states topic to store the current joint states for use within the class
        '''
        self.joint_state = joint_state

    def state_manager(self):
        """
        Pushes the next state
        """
        self.current_state_index +=1
        self.current_state = self.states[self.current_state_index]
    
    def moveBase(self):      
        if self.base_to_tag_angle >= 0.12:
            self.base_commands.spin(negative=False)
        elif self.base_to_tag_angle <= -0.12:
            self.base_commands.spin(negative=True)
        elif self.base_to_tag_distance > 0.3:
            self.base_commands.move_forward()
        if self.base_to_tag_distance <= 0.4:
            self.state_manager()



    #Droping off functions
    def align_to_surface(self):
        rospy.loginfo('align_to_surface')
        trigger_request = TriggerRequest() 
        trigger_result = self.trigger_align_with_nearest_cliff_service(trigger_request)
        rospy.loginfo('trigger_result = {0}'.format(trigger_result))

    def tableCommand(self):
        if self.current_state == "table":
            #self.align_to_surface()
            length = 0.1
            hold_angle = 0.2
            drop_angle = -0.9
            max_extend = 1
            forward_wrist = 1.65
            out_wrist = 0
            shape  = [[0,0.5,forward_wrist,hold_angle], [0,0.9,forward_wrist,hold_angle], [0.3,0.9,out_wrist,hold_angle],[0.3,0.9,out_wrist,drop_angle]]
            self.issue_multipoint_command(shape)
            shape.reverse()
            rospy.sleep(1)
            self.issue_multipoint_command(shape)
            rospy.sleep(1)
            self.state_manager()


    def handleTransforms(self, tag_name):
            """
            Gets transform vectors from aruco to base and aruco to camera, computes angle between them. Publishes transforms and camera angle needed
            to center on aruco tag 
            """
            # Get transforms from the tag to the camera and base
            base_to_tag:TransformStamped = self.tf_buffer.lookup_transform('base_link', tag_name, rospy.Time(0))
            
            #Brian - There was a issue where it was more diffidult to get the angle while using the camera link as the source
            cam_to_tag:TransformStamped = self.tf_buffer.lookup_transform('camera_link', tag_name, rospy.Time(0))
            
            if (self.last_transform != base_to_tag.transform):
                self.count =0
                rospy.loginfo("Found Tag")
                self.search_flag = False
                # Get angle between the tag and the camera
                cam_hypotonuse = (cam_to_tag.transform.translation.x ** 2 + cam_to_tag.transform.translation.y ** 2) ** 0.5
                self.cam_to_tag_angle = -math.acos(base_to_tag.transform.translation.x/cam_hypotonuse)
                # Get angle between the tag and the base
                self.base_to_tag_angle = math.atan2(base_to_tag.transform.translation.y, base_to_tag.transform.translation.x)
                # Get distance between the tag and the base
                self.base_to_tag_distance = (base_to_tag.transform.translation.x ** 2 + base_to_tag.transform.translation.y ** 2) ** 0.5
                
                self.last_transform = base_to_tag.transform
            else:
                self.count+=1
                if self.count > 10:
                    raise tf2_ros.LookupException
            
    def find_tag(self):
        '''  
        Sees if aruco tag is in frame and acts accordingly
        '''     
        #Check if tag is in view
        try:
            tag_name = self.current_state
            self.handleTransforms(tag_name)
            self.search_flag = False
        #Tag not found
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Problem finding tag")
            if "nav" in self.current_state:
                self.search_flag = True
            pass

#Camera Functions
    def pan_follower(self):
        angle = self.base_to_tag_angle
        rospy.loginfo("Pan to %s radians", angle)
        new_pose = {'joint_head_pan': angle}
        self.move_to_pose(new_pose)
        self.last_pan_camera_angle = angle 

    def tilt_follower(self):
        angle = self.cam_to_tag_angle
        new_pose = {'joint_head_tilt': angle}
        self.move_to_pose(new_pose)
        self.last_camera_angle = angle


    def update_camera_angle(self):
        if not self.search_flag:
            self.tilt_follower()
            self.pan_follower()
        else:
            self.camera_search()

    def camera_search(self):
        joint_index = self.joint_state.name.index('joint_head_pan')
        joint_value = self.joint_state.position[joint_index]
        if not -4 < (self.delta + joint_value) < 1.3:
            self.delta = -self.delta
        command = {'joint': 'joint_head_pan', 'delta': self.delta}
        self.send_command(command)

    #Joiint control 
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

    def createPath(self, shape):
        converted_shape = []
        for point in shape:
            trajectory = JointTrajectoryPoint()
            trajectory.positions = [point[0], point[1],point[2], point[3]]
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
        trajectory_goal.trajectory.points = self.createPath(shape)

        # Specify the coordinate frame that we want (base_link) and set the time to be now
        trajectory_goal.trajectory.header.stamp = rospy.Time(0.0)
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        # Make the action call and send the goal. The last line of code waits
        # for the result before it exits the python script
        self.trajectory_client.send_goal(trajectory_goal)
        rospy.loginfo('Sent list of goals = {0}'.format(trajectory_goal))
        self.trajectory_client.wait_for_result()


    def main(self):
        hm.HelloNode.main(self, 'save_pose', 'save_pose', wait_for_first_pointcloud=False)
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        self.r = rospy.Rate(rospy.get_param('~rate', 10.0))

        #rospy.wait_for_service('/funmap/trigger_align_with_nearest_cliff')
        #rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_align_with_nearest_cliff.')
        #self.trigger_align_with_nearest_cliff_service = rospy.ServiceProxy('/funmap/trigger_align_with_nearest_cliff', Trigger)


        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rate = rospy.Rate(10) # 10hz
        #Set intial pose of camera
        pose = {'joint_head_tilt': -math.pi/4, 'joint_head_pan': 0}
        self.move_to_pose(pose)
        while not rospy.is_shutdown():
            self.find_tag()
            self.update_camera_angle()
            if not self.search_flag and "nav" in self.current_state:
                self.moveBase()
            self.tableCommand()
            rospy.loginfo("Current state %s", self.current_state)
            rate.sleep()
         
if __name__ == '__main__':
        
    try:
        node = ReplicationController()
        node.main()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
