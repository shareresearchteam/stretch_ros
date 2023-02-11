#Remember to chmod and add to the CMake? 
import hello_misc as hm

import rospy
import tf2_ros
from math import pi, sqrt, atan2

from std_srvs.srv import Trigger

from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import JointState
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

        #Controls how many duplicates it will tolerate efore defaulting
        self.count = 0

        #Initial data for publishing
        self.cam_to_tag_angle = -math.pi/4
        self.base_to_tag_angle = 0
        self.base_to_tag_distance = 0

        self.cam_to_tag_angle_publisher = rospy.Publisher('cam_to_tag_angle', Float32, queue_size=1)
        self.base_to_tag_angle_publisher = rospy.Publisher('base_to_tag_angle', Float32, queue_size=1)
        self.base_to_tag_distance_publisher = rospy.Publisher('base_to_tag_distance', Float32, queue_size=1)
        self.next_state_subscriber = rospy.Subscriber('actions/current_state', StateMessage, self.next_state_callback)

        self.last_transform = Transform()

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
            # Get transforms from the tag to the camera and base
            base_to_tag:TransformStamped = self.tf_buffer.lookup_transform('base_link', tag_name, rospy.Time(0))
            base_to_cam:TransformStamped = self.tf_buffer.lookup_transform('base_link', 'camera_link', rospy.Time(0))
            #Brian - There was a issue where it was more diffidult to get the angle while using the camera link as the source
            cam_to_tag:TransformStamped = self.tf_buffer.lookup_transform('camera_link', tag_name, rospy.Time(0))

            if (self.last_transform != base_to_tag.transform):
                self.count =0
                rospy.loginfo("Found Tag")
                # Get angle between the tag and the camera
                cam_hypotonuse = (cam_to_tag.transform.translation.x ** 2 + cam_to_tag.transform.translation.y ** 2) ** 0.5
                self.cam_to_tag_angle = -math.acos(base_to_tag.transform.translation.x/cam_hypotonuse)
                # Get angle between the tag and the base
                self.base_to_tag_angle = math.atan2(base_to_tag.transform.translation.y, base_to_tag.transform.translation.x)
                # Get distance between the tag and the base
                self.base_to_tag_distance = (base_to_tag.transform.translation.x ** 2 + base_to_tag.transform.translation.y ** 2) ** 0.5
                
                self.last_transform = base_to_tag.transform
            else:
                self.count +=1
                if self.count >= 7:
                    # The tf is old; ignore it
                    self.cam_to_tag_angle = -math.pi/4
                    self.base_to_tag_angle = 0
                    self.base_to_tag_distance = 0

            # Publish all data
            self.cam_to_tag_angle_publisher.publish(self.cam_to_tag_angle)
            self.base_to_tag_angle_publisher.publish(self.base_to_tag_angle)
            self.base_to_tag_distance_publisher.publish(self.base_to_tag_distance)

    def find_tag(self):
        '''  
        Sees if aruco tag is in frame and acts accordingly
        '''     
        #Check if tag is in view
        try:
            # tag_name = self.next_state.name
            self.handleTransforms("table")
        #Tag not found
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            
            rospy.loginfo("Problem finding tag")
            #Do a search to find the tag
            
            pass
        #except AttributeError:
        #    rospy.loginfo("State not detected")
        #    pass

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
        rate = rospy.Rate(10) # 10hz
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
