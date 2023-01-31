#Remember to chmod and add to the CMake? 
import hello_misc as hm

import rospy
import logging
import tf
import tf2_ros
import time
from math import pi, sqrt, atan2
import json 
import os 

from std_srvs.srv import Trigger

import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint 
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import navigation as nav
import math 
from std_msgs.msg import Float32
import numpy as np 


class ArucoNavigationNode(hm.HelloNode):
    def __init__(self):

        super().__init__()
        
        self.translation = None
        self.rotation = None 
        self.joint_state = None 
        self.file_path = rospy.get_param('/file_path')
        self.transform_pub = rospy.Publisher('ArUco_transform', TransformStamped, queue_size=1)
        self.camera_angle_publisher = rospy.Publisher('Camera_Angle', Float32, queue_size=1)
       

        #Attempt to access current saved poses in saved_poses.json, if fail set to empty
        try:
            saved_file = open(self.file_path + '/saved_poses.json')
            self.pose_dict = json.load(saved_file)
            saved_file.close()
        except:
            self.pose_dict = {}
        
        self.main() 

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

    def find_tag(self, requested_tag):
        '''
        Pans the head at three tilt angles to search for the requested frame (usually either the name of an aruco tag or "map"). Cycles up for up to two full searches (a total of 6 rotations) before timing 
        out. If the frame is found, a tf_listener finds the pose of the base_link in the requested frame and saves it in the translation and rotation variables for use in the next functions.
        '''    
        min_rotation = -2
        max_rotation = 1.53
        num_steps = 10
        step = abs(min_rotation - max_rotation) / num_steps

        #Set intial pose of camera
        #pose = {'joint_head_tilt': -2*pi/4, 'joint_head_pan': min_rotation}
        #self.move_to_pose(pose)
        #Breaks out of while
        found_tag = False
        #Limits time
        count = 0

        while not found_tag:
            command = {'joint': 'joint_head_pan', 'delta': step}
            self.send_command(command)
            rospy.sleep(0.5)

            try:
                rospy.loginfo('LOOKING FOR THIS TAG: ')
                rospy.loginfo(requested_tag)
                #Class variables stored here 
                self.translation, self.rotation = self.tf_listener.lookupTransform(requested_tag, 'base_link', rospy.Time(0))
                rospy.loginfo("Found Requested Tag")
                found_tag = True

            except:
                
                # Check if the head has completed a full rotation
                if self.joint_state.position[self.joint_state.name.index('joint_head_pan')] > (max_rotation - step):
                    
                    pose = {'joint_head_pan': min_rotation}
                    self.move_to_pose(pose)

                    # After a full head rotation, change the head tilt 
                    if self.joint_state.position[self.joint_state.name.index('joint_head_tilt')] >= -0.1:
                        pose = {'joint_head_tilt': -1*pi/4}
                        self.move_to_pose(pose)
                        count += 1
                    else:
                        command = {'joint': 'joint_head_tilt', 'delta': pi/8}
                        self.send_command(command)

                    time.sleep(.5)
    
            if count >= 2:
                rospy.loginfo("Timed Out Looking for Tag")
                # self.switch_to_navigation_mode()
                return False

        # self.switch_to_navigation_mode()
        return True

    def find_tag_one_angle(self, tag_name):
        '''magX = x1*x1 + x2*x2 + x3*x3 
        Pans the head at three tilt angles to search for the requested frame (usually either the name of an aruco tag or "map"). Cycles up for up to two full searches (a total of 6 rotations) before timing 
        out. If the frame is found, a tf_listener finds the pose of the base_link in the requested frame and saves it in the translation and rotation variables for use in the next functions.
        '''    
        min_rotation = -2
        max_rotation = 1.53
        num_steps = 10
        step = abs(min_rotation - max_rotation) / num_steps

        #Set intial pose of camera
        #pose = {'joint_head_tilt': -pi/4, 'joint_head_pan': 0}
        #self.move_to_pose(pose)
        #Breaks out of while
        #found_tag = False
        #Limits time
        count = 0

        #Check if tag is in view
        try:
            transform = self.tf_buffer.lookup_transform(tag_name,'base_link',rospy.Time(0))
            camera_transform = self.tf2_buffer.lookup_transform(tag_name,'camera_link', rospy.Time(0))
            print(transform)
            print(camera_transform)
            camera_angle = angle_between((transform.transform.translation.x,0),
                                         (camera_transform.transform.translation.x,camera_transform.transform.translation.z,))
            #self.angle_solver(transform.transform, camera_transform.transform)

            rospy.loginfo("Desired camera angle %s", camera_angle)
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            #rospy.loginfo("Found Requested Tag: \n%s", transform)
            self.camera_angle_publisher.publish(camera_angle)
            self.transform_pub.publish(transform)
	   
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Problem finding tag")    
            pass

        # self.switch_to_navigation_mode()
        return None

    def angle_solver(self, transform_one, transform_two):
        x = transform_one.translation
        x1 = x.x
        x2 = x.y
        x3 = x.z
        y = transform_two.translation
        y1 = y.x
        y2 = y.y
        y3 = y.z
        dotProduct = x1*y1 + x2*y2 + x3*y3 
        magX = x1*x1 + x2*x2 + x3*x3 
        magY = y1*y1 + y2*y2 + y3*y3
        cosine = dotProduct/ (magX*magY)
        return pi/2 - math.acos(cosine)


    def save_pose(self, pose_name, aruco_name):
        '''
        Looks for the requested frame (the name of an aruco tag or "map") then returns the translation and rotation found by the tf_listener in find_tag as a pose
        in the requested frame.
        '''
        if self.find_tag(aruco_name):
        
            #Create new PoseStamped msg
            msg = PoseStamped()
            msg.header.frame_id = aruco_name
            
            msg.pose.position.x = self.translation[0]
            msg.pose.position.y = self.translation[1]
            msg.pose.position.z = self.translation[2]

            msg.pose.orientation.x = self.rotation[0]
            msg.pose.orientation.y = self.rotation[1]
            msg.pose.orientation.z = self.rotation[2]
            msg.pose.orientation.w = self.rotation[3]

            saved_file = open(self.file_path + "/saved_poses.json","w")
            new_pose = self.msg_to_object(msg)
            self.pose_dict[pose_name.lower()] = new_pose.serialize()
            json.dump(self.pose_dict,saved_file)
            saved_file.close()

            return True

        else: 
            rospy.loginfo("Could not save pose")
            return False

    def go_to_pose(self, pose_name):
        '''
        Finds the requested pose in the saved pose dictionary, and sends a move_base goal to return to the given pose.
        '''
        #Default positioning
        pose = {'wrist_extension': 0.01} 
        self.move_to_pose(pose)
        pose = {'joint_wrist_yaw': 3.3}
        self.move_to_pose(pose)
        pose = {'joint_lift': 0.35}
        self.move_to_pose(pose)
        
        #Find relevant tag
        pose_goal = self.find_tag_one_angle("nav_1")
        print(type(pose_goal))
        if pose_goal is None:
            rospy.loginfo("Aruco not found")

        self.navstack.go_to(pose_goal.transform.translation.x. pose_goal.transform.translation.y, 1.2)
        

    
    def main(self):
        hm.HelloNode.main(self, 'save_pose', 'save_pose', wait_for_first_pointcloud=False)

        self.r = rospy.Rate(rospy.get_param('~rate', 15.0))

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        #self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        self.switch_to_position_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_to_navigation_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            self.find_tag_one_angle("nav_3")
            rate.sleep()

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
         
if __name__ == '__main__':
    
    node = ArucoNavigationNode()
    node.main()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
