#!/usr/bin/python3

import rospy
from std_msgs.msg import String, Bool
from hello_helpers.msg import StateMessage
import datetime
from state import State


class StateController():
    def __init__(self):
        rospy.init_node('state_controller')
        self.current_state_subscriber = rospy.Subscriber('actions/current_state', StateMessage, self.update_state)
        self.current_state_publisher = rospy.Publisher('actions/current_state', StateMessage, queue_size=10)
        self.r = rospy.Rate(self.rate)
        rospy.loginfo("Starting State Controller Node")

        self.current_state = 0
        self.states = [State("nav_1", 0, 1), State("nav_2", 0, 1), State("nav_3", 0, 1), State("table", 0, 1)]
        self.rate = 30

    def update_state(self, msg):
        """
        Checks to see if sent destination was reached, gives robot new goal
        """
        receivedState = State.fromMsg(msg)
        #Send some command to joint to look for next tag 
        #Current State is the same as what was the last next state
        if self.current_state.name == receivedState.name and receivedState.completed:
            self.current_state = self.state_manager()
    
    def state_manager(self):
        """
        Pushes the next state
        """
        return self.states.pop()

    def spin(self):
        while not rospy.is_shutdown():
            self.current_state_publisher.publish(self.current_state.toMsg())
            self.r.sleep()

if __name__ == '__main__':
    try:
        node = StateController()
        node.spin()

    except KeyboardInterrupt:
        rospy.signal_shutdown()
        print('interrupt received, so shutting down')
