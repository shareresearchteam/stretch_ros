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
from hello_helpers.msg import StateMessage
from state import State 


class ArucoNavigationNode(hm.HelloNode):
    def __init__(self):

        super().__init__()
        
        self.translation = None
        self.rotation = None 
        self.joint_state = None 
        self.next_state = None 

        self.transform_pub = rospy.Publisher('ArUco_transform', TransformStamped, queue_size=1)
        self.base_angle_publisher = rospy.Publisher('Base_Angle', Float32, queue_size=1)
        self.camera_angle_publisher = rospy.Publisher('Camera_Angle', Float32, queue_size=1)
        self.next_state_subscriber = rospy.Subscriber('actions/next_state', StateMessage, self.next_state_callback)


    def next_state_callback(self, msg:StateMessage):
        """"
        Callback for the next_state topic to store current state information
        """
        #rospy.loginfo("Updating Next State")
        self.next_state = State.fromMsg(msg)

    def handleTransforms(self, tag_name):
            """
            Gets transform vectors from aruco to base and aruco to camera, computes angle between them. Publishes transforms and camera angle needed
            to center on aruco tag 
            """
            #Get vectors to point from camera and base
            center_base_transform = self.tf_buffer.lookup_transform('centered_base_link','base_link', rospy.Time(0))
            base_transform:TransformStamped = self.tf_buffer.lookup_transform(tag_name,'centered_base_link',rospy.Time(0))
            camera_transform:TransformStamped = self.tf_buffer.lookup_transform(tag_name,'camera_link', rospy.Time(0))
            #Get angle between vectors

            camera_angle = angle_between((base_transform.transform.translation.x,0),
                                         (camera_transform.transform.translation.x,camera_transform.transform.translation.z,))
            #The angle the base needs to rotate to move towards
            
            center_angle = math.atan(base_transform.transform.translation.y/base_transform.transform.translation.x)
            rospy.loginfo("Translation x: %s", base_transform.transform.translation.x)
            rospy.loginfo("Translation y: %s", base_transform.transform.translation.y)
            rospy.loginfo("Translation z: %s", base_transform.transform.translation.z)
            
            rospy.loginfo("center_angle %s", base_transform.transform.rotation.z)
            #Publish camera angle and distances/rotations for other nodes
            self.camera_angle_publisher.publish(camera_angle)
            self.transform_pub.publish(base_transform)

    def find_tag(self):
        '''  
        Sees if aruco tag is in frame and acts accordingly
        '''     
        #Check if tag is in view
        try:
            tag_name = self.next_state.name
            self.handleTransforms(tag_name)
        #Tag not found
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            fakeTransform = TransformStamped()
            fakeTransform.transform.translation.x = 0
            fakeTransform.transform.rotation.z = 0
            self.transform_pub.publish(fakeTransform)
            rospy.loginfo("Problem finding tag")    
            pass
        except AttributeError:
            rospy.loginfo("State not detected")
            pass

        return None 

    def main(self):
        hm.HelloNode.main(self, 'save_pose', 'save_pose', wait_for_first_pointcloud=False)

        self.r = rospy.Rate(rospy.get_param('~rate', 10.0))

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        self.switch_to_position_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_to_navigation_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
        rate = rospy.Rate(1) # 10hz
        while not rospy.is_shutdown():
            self.find_tag()
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
        
    try:
        node = ArucoNavigationNode()
        node.main()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
