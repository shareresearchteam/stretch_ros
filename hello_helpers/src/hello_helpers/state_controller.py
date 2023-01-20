import rospy
from std_msgs.msg import String, Bool


class StateController():
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
        if msg.data != self.current_state and not self.in_progress:
            #Update the in progress check
            self.next_state_publisher.publish(state_manager())



    def current_state_callback(self, msg):
        self.current_state = msg.data 
        rospy.loginfo("Current state is %s", self.current_state)

    def in_progress_callback(self, msg):
        self.in_progress = msg.data
    
    def state_manager(self, state):
        """
        Pushes the next state
        """
        match state:
            case "first":
                self.nav_check_publisher.publish(True)
                return "second"
            case "second":
                self.nav_check_publisher.publish(True)
                return "third"
            case _:
                self.nav_check_publisher.publish(False)
                return "table"
        

    def main(self):

        self.current_state_subscriber = rospy.Subscriber('current_state', String, self.update_state)
        self.in_progress_subscriber = rospy.Subscriber('in_progress', Bool, self.in_progress_callback)


        self.next_state_publisher = rospy.Publisher('next_state', String, queue_size=10)
        self.nav_check_publisher = rospy.Publisher('nav', String, queue_size=10)


        rospy.init_node('state_controller')
        r   = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():
            self.pub.publish()
            r.sleep()


