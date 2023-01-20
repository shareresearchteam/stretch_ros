import rospy
from std_msgs.msg import String, Bool
import aruco_navigation as an  

class QRNavigator(hm.HelloNode):

    def __init__(self):

        #Misc
        super.__init__()
        self.rate = 10
        self.arucoNav = an.ArucoNavigaation()
        self.current_state = None
        self.next_state = None
        self.in_progrss = False
        
        #Information to publish for controller
        self.current_state_publisher = rospy.Publisher('current_state', String, queue_size=10)
        self.in_progress_publisher = rospy.Publisher('in_progress', Bool, self.updateState)
        
        #Information from State Controller 
        self.next_state_subscriber = rospy.Subscriber('next_state', String, next_state_callback)
        self.nav_check = rospy.Subscriber('nav', Bool, queue_size=self.rate)
        


    def next_state_callback(self, msg):
        """
        Handle updates to the next state from State Controller
        msg - Next State Message
        """
        #If its not already going somewhere
        if self.current_state != msg.data and self.nav_check and not in_progress:
            #Set the next state and navigate to it
            self.next_state = msg.data
            #Tell system we are now in motion
            self.in_progress = True
            self.in_progress_publisher.publish(self.in_progress)
            rospy.loginfo("Issued new movement command to %s", self.next_state)
            success = not self.arucoNav.go_to_pose(self.next_state)
            if success:
                rospy.loginfo("Command complete")
                self.in_progress = False
                self.in_progress_publisher.publish(self._in_progress)
                self.current_state = self.next_state
        else:
            rospy.loginfo("Command ")
            




    def update_state(self):
        self.next_state 

    def current_state_callback(self, data):
        self.current_state = data.data 

    def in_progress_callback(self, data):
        self.next_state = data.data
    

    def main(self):
        rospy.init_node('qr_nav')
        r   = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():
            r.sleep()
