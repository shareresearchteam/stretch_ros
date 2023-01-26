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


class Pose():
    """
    Stores and converts PoseStamped messages 
    """
    def __init__(self, **kwargs):
        self.id = kwargs.get("id")
        #Positional attributes
        self.pos_x = kwargs.get("pos_x")
        self.pos_y = kwargs.get("pos_y")
        self.pos_z = kwargs.get("pos_z")
        #Orientation attributes
        self.ori_x = kwargs.get("ori_x")
        self.ori_y = kwargs.get("ori_y")
        self.ori_z = kwargs.get("ori_z")
        self.ori_w = kwargs.get("ori_w")

    @classmethod
    def from_dictionary(cls, dict):
        return cls(id=dict[0], pos_x=dict[1],pos_y=dict[2],
        pos_z=dict[3],ori_x=dict[4],ori_y=dict[5], ori_z=dict[6],
        ori_w=dict[7])

    def poseMsg(self):
        msg = PoseStamped()
        msg.header.frame_id = self.id
            
        msg.pose.position.x = self.pos_x
        msg.pose.position.y = self.pos_y
        msg.pose.position.z = self.pos_z

        msg.pose.orientation.x = self.ori_x
        msg.pose.orientation.y = self.ori_x
        msg.pose.orientation.z = self.ori_x
        msg.pose.orientation.w = self.ori_x
        return msg

    def serialize(self):
        return [self.id,
            self.pos_x,
            self.pos_y,
            self.pos_z,
            self.ori_x,
            self.ori_y,
            self.ori_z,
            self.ori_w
        ]

class ArucoNavigationNode(hm.HelloNode):
    def __init__(self):

        super().__init__()
        
        self.translation = None
        self.rotation = None 
        self.joint_state = None 
        self.file_path = rospy.get_param('/file_path')
        self.transform_pub = rospy.Publisher('ArUco_transform', TransformStamped, queue_size=10)

        #Attempt to access current saved poses in saved_poses.json, if fail set to empty
        try:
            saved_file = open(self.file_path + '/saved_poses.json')
            self.pose_dict = json.load(saved_file)
            saved_file.close()
        except:
            self.pose_dict = {}
        
        self.main() 

    def msg_to_object(self, msg):
        """
        Converts PoseStamped message into a Pose object (to be later saved as dictionary entry)
        input: pose - PostStamped msg
        """
        return Pose(id = msg.header.frame_id, pos_x = msg.pose.position.x, pos_y =msg.pose.position.y,
              pos_z = msg.pose.position.z, ori_x = msg.pose.orientation.x, ori_y = msg.pose.orientation.y, 
              ori_z = msg.pose.orientation.z, ori_w = msg.pose.orientation.w)
    
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
        pose = {'joint_head_tilt': -2*pi/4, 'joint_head_pan': min_rotation}
        self.move_to_pose(pose)
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
                self.translation, self.rotation = self.tf_listener.lookupTransform('base_link', requested_tag, rospy.Time(0))
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
        '''
        Pans the head at three tilt angles to search for the requested frame (usually either the name of an aruco tag or "map"). Cycles up for up to two full searches (a total of 6 rotations) before timing 
        out. If the frame is found, a tf_listener finds the pose of the base_link in the requested frame and saves it in the translation and rotation variables for use in the next functions.
        '''    
        min_rotation = -2
        max_rotation = 1.53
        num_steps = 10
        step = abs(min_rotation - max_rotation) / num_steps

        #Set intial pose of camera
        pose = {'joint_head_tilt': -2*pi/4, 'joint_head_pan': 0}
        self.move_to_pose(pose)
        #Breaks out of while
        found_tag = False
        #Limits time
        count = 0


        #Check if tag is in view
        try:
            transform = self.tf_buffer.lookup_transform('base_link',
                                                                tag_name,
                                                                rospy.Time())
            rospy.loginfo("Found Requested Tag: \n%s", transform)
            self.transform_pub.publish(transform)
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

        # self.switch_to_navigation_mode()
        return None



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
    
    def main(self):
        hm.HelloNode.main(self, 'save_pose', 'save_pose', wait_for_first_pointcloud=False)

        self.r = rospy.Rate(rospy.get_param('~rate', 15.0))

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)


        self.tf_listener = tf.TransformListener()
        self.switch_to_position_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_to_navigation_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()


          
if __name__ == '__main__':
    
    node = ArucoNavigationNode()
    node.main()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')