import rospy 
from hello_helpers.msg import StateMessage
import datetime


class State():
    def __init__(self, name:str, completed:int, navigation:int):
        self.name = name
        self.completed = completed
        self.time = datetime.datetime.now()
        self.navigation = navigation

    @classmethod
    def fromMsg(cls, msg):
        if msg is not None:
            return cls(msg.name, msg.completed, msg.navigation) 
        else:
            rospy.loginfo("No message received")
            return None

    def updateComplete(self):
        self.completed = 1


    def __eq__(self, __o: object) -> bool:
        return self.name ==__o.name and self.completed == __o.check
    
    def __str__(self) -> str:
        return "State: {} \n name: {} \n check: {}".format(self.time, self.name, self.completed)
    
    def toMsg(self):
        msg = StateMessage()
        msg.name = self.name
        msg.completed = self.completed
        msg.navigation = self.navigation
        return msg 
        
    