import rospy
from std_msgs.msg import String, Bool
import get_aruco_info as an  

class QRNavigator(hm.HelloNode):

    def __init__(self):

        #Misc
        super.__init__()
        self.rate = 10
        self.arucoNav = an.ArucoNavigaation()
        self.current_state = None
        self.next_state = None
        self.in_progrss = False
        self.nav = None
        
        #Information to publish for controller
        self.current_state_publisher = rospy.Publisher('current_state', String, queue_size=10)
        self.in_progress_publisher = rospy.Publisher('in_progress', Bool, self.updateState)
        

        
    def nav_callback(self, msg):
        self.nav = msg.data


    def next_state_callback(self, msg):
        """
        Handle updates to the next state from State Controller
        msg - Next State Message
        """
        #Set the next state and navigate to it
        self.next_state = msg.data
        #If its not already going somewhere
        if self.current_state != msg.data and self.nav_check and not self.in_progress:
            self.attemptMove()
        else:
            if self.in_progress:
                rospy.loginfo("Stretch is moving towards: %s", self.next_state)
            else:
                rospy.loginfo("Arrived at target location: %s", self.next_state)
            
    def attemptMove(self):
            """"
            Helper function for next_state_callback. Issues move command to State Controller and 
            updates private variables
            """
            #Tell system we are now in motion
            self.in_progress = True
            self.in_progress_publisher.publish(self.in_progress)
            rospy.loginfo("Issued new movement command to %s", self.next_state)
            success = not self.arucoNav.go_to_pose(self.next_state)
            #Revert in_progress, set current_state
            if success:
                rospy.loginfo("Completed move")
                self.in_progress = False
                self.in_progress_publisher.publish(self._in_progress)
                self.current_state = self.next_state




    def update_state(self):
        self.next_state 

    def current_state_callback(self, data):
        self.current_state = data.data 

    def in_progress_callback(self, msg):
        self.next_state = msg.data
    

    def main(self):
        #Information from State Controller 
        rospy.Subscriber('next_state', String, self.next_state_callback)
        rospy.Subscriber('nav', Bool, self.nav_callback)

        rospy.init_node('qr_nav')
        r   = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    try:
        node = QRNavigator()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')