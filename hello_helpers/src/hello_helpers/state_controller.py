#!/usr/bin/python3

import rospy
from std_msgs.msg import String, Bool, Int8
from hello_helpers.msg import StateMessage
import datetime
from state import State


class StateController():
    def __init__(self):
        rospy.init_node('state_controller')
        self.current_state_publisher = rospy.Publisher('actions/current_state', StateMessage, queue_size=10)
        self.flag_publisher = rospy.Publisher('actions/flag', Int8, queue_size=10)
        self.flag_subscriber = rospy.Subscriber('actions/flag', Int8, self.flag_callbak)
        
        self.r = rospy.Rate(20)
        rospy.loginfo("Starting State Controller Node")

        self.flag = 0
        self.current_state = State("nav_1", 0, 1)
        self.current_state_index = 0
        self.states = [State("nav_1", 0, 1), State("nav_2", 0, 1), State("nav_3", 0, 1), State("table", 0, 1)]
        #self.rate = 30
    
    def flag_callbak(self, msg):
        """
        Updates state, on the rise of actions/flag topic
        """
        if self.flag ==0 and msg.data == 1:
            rospy.loginfo("Index %s", self.current_state_index)
            self.current_state = self.state_manager()
            rospy.loginfo("Testing callback")
            #self.flag_publisher.publish(0)
        self.flag = msg.data

    def state_manager(self):
        """
        Pushes the next state
        """
        self.current_state_index +=1
        return self.states[self.current_state_index]

    def spin(self):
        while not rospy.is_shutdown():
            # Publish current state
            self.current_state_publisher.publish(self.current_state.toMsg())
            self.r.sleep()

if __name__ == '__main__':
    try:
        node = StateController()
        node.spin()

    except KeyboardInterrupt:
        rospy.signal_shutdown()
        print('interrupt received, so shutting down')
