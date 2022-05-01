#!/usr/bin/env python
from ast import Pass
import rospy
from Motor import MotorManager
from RobotState import RobotState, ContorlMode


class LegState(RobotState):
    
    def __init__(self):
        RobotState.__init__(self, outcomes=["Transform"])
        self.motorControlMode = ContorlMode.POS_MODE
        self.transformLength = 10.0
    
    
        
    def execute(self, userdata):
        
        self.stateChangeFlag = False
        
        r = rospy.Rate(100)
        
        while(not self.stateChangeFlag):
        
            if self.joyData is not None:
                pass
        
            self.sendData()
            
            r.sleep()
        
        return "Transform"
        
    