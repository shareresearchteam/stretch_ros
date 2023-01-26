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

class Controller():
    def __init__(self):
        self.current_state = None
        self.next_state = "first"
        self.nav_check = False
        self.in_progress = False
        self.rate = 10

    def update_state(self, msg):
        """
        Checks to see if sent destination was reached, gives robot new goal
        """
        if 
        

    def main(self):

        self.trasform_subscriber = rospy.Subscriber('ArUco_transform', TransformStamped, self.update_state)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        self.switch_to_position_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_to_navigation_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)



        rospy.init_node('state_controller')
        r   = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():
            self.pub.publish()
            r.sleep()