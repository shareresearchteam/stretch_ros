import rospy
from std_msgs.msg import String, Bool
from hello_helpers.msg import StateMessage
import datetime
from state import State


class StateController():
    def __init__(self):
        self.current_state = None
        self.next_state = State("nav_1", 0)
        self.rate = 10

    def update_state(self, msg):
        """
        Checks to see if sent destination was reached, gives robot new goal
        """
        receivedState = State.fromMsg(msg)
        #Check if State is different
        if receivedState != self.current_state:
            #Update the recorded state and publish it
            self.current_state = receivedState
            rospy.loginfo("State Updated To: %s", self.current_state)
            
            if receivedState.check:
                self.next_state = self.state_manager(msg.name)
                self.next_state_publisher.publish(self.next_state.toMsg())
                rospy.loginfo("Now Executing To: %s", self.next_state)

    
    def state_manager(self, state_name):
        """
        Pushes the next state
        """
        if state_name == "nav_1":
            return State("nav_2", 0)
        elif state_name == "nav_2":
            return State("nav_3", 0)
        else:
            return State("table", 0)
        

    def main(self):

        self.current_state_subscriber = rospy.Subscriber('actions/current_state', StateMessage, self.update_state)
        self.next_state_publisher = rospy.Publisher('actions/next_state', StateMessage, queue_size=10)


        rospy.init_node('state_controller')
        r   = rospy.Rate(self.rate) 
        #self.update_state(None)
        while not rospy.is_shutdown():
            #rospy.loginfo('Is running')
            self.next_state_publisher.publish(self.next_state.toMsg())
            #rospy.loginfo("Next state is %s", self.next_state)
            r.sleep()

if __name__ == '__main__':
    

    try:
        node = StateController()
        node.main()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
