import rospy 
from hello_helpers.msg import StateMessage
import datetime


class State():
    def __init__(self, name:str, check:int):
        self.name = name
        self.check = check
        self.time = datetime.datetime.now()

    @classmethod
    def fromMsg(cls, msg):
        if msg is not None:
            return cls(msg.name, msg.check) 
        else:
            rospy.loginfo("No message received")
            return None

    def __eq__(self, __o: object) -> bool:
        return self.name ==__o.name and self.check == __o.check
    
    def __str__(self) -> str:
        return "State: {} \n name: {} \n check: {}".format(self.time, self.name, self.check)
    
    def toMsg(self):
        msg = StateMessage()
        msg.name = self.name
        msg.check = self.check 
        return msg 
        
    