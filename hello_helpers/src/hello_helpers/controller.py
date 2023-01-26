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


    def go_to_pose(self, pose_name):
        '''
        Finds the requested pose in the saved pose dictionary, and sends a move_base goal to return to the given pose.
        '''

        if pose_name in self.pose_dict:

            pose = {'wrist_extension': 0.01} 
            self.move_to_pose(pose)

            pose = {'joint_wrist_yaw': 3.3}
            self.move_to_pose(pose)
            
            pose = {'joint_lift': 0.22}
            self.move_to_pose(pose)
            
            pose_goal = Pose.from_dictionary(self.pose_dict[pose_name]).poseMsg()
            tag = pose_goal.header.frame_id 
            if not self.find_tag(tag):
                print("Could not find tag")
                return False

            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)

            map_goal = MoveBaseGoal()
            while True:
                try:
                    map_goal.target_pose = tf_buffer.transform(pose_goal, 'map', rospy.Duration(0))
                    break
                except: 
                    if not self.find_tag(tag):
                        print("Could not find tag")
                        return False 
            
            map_goal.target_pose.pose.position.z = 0.0
            eul = tf.transformations.euler_from_quaternion((map_goal.target_pose.pose.orientation.x,
             map_goal.target_pose.pose.orientation.y,
             map_goal.target_pose.pose.orientation.z, map_goal.target_pose.pose.orientation.w))
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, eul[2])
            map_goal.target_pose.pose.orientation.x = quat[0]
            map_goal.target_pose.pose.orientation.y = quat[1]
            map_goal.target_pose.pose.orientation.z = quat[2]
            map_goal.target_pose.pose.orientation.w = quat[3]
            rospy.loginfo(map_goal)
            self.client.send_goal_and_wait(map_goal)
            rospy.loginfo("DONE!")

            return True
        else:
            print("Pose not found")
            return False



    def update_state(self, msg):
        """
        Checks to see if sent destination was reached, gives robot new goal
        """
        
        

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