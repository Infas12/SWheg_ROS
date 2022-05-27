#!/usr/bin/env python
from ast import Pass
import rospy
from States.Motor import MotorManager
from RobotState import RobotState, ContorlMode


class LegState(RobotState):
    
    def __init__(self):
        RobotState.__init__(self, outcomes=["Transform"])
        self.motorControlMode = ContorlMode.POS_MODE
        self.IsLeggedMode = True 
        self.tick = 0
        self.period = 2000 # gait cycle
    
    def execute(self, userdata):
        
        self.stateChangeFlag = False
        self.tick = 0
        r = rospy.Rate(1000)
        
        while(not self.stateChangeFlag):
            
            self.tick += 1
            
            MotorManager.instance().getMotor("LF_Joint").positionSet = -self.getPosTrot(self.tick + 0.5*self.period,self.period)
            MotorManager.instance().getMotor("LB_Joint").positionSet = -self.getPosTrot(self.tick,self.period)
            MotorManager.instance().getMotor("RF_Joint").positionSet =  self.getPosTrot(self.tick,self.period)
            MotorManager.instance().getMotor("RB_Joint").positionSet =  self.getPosTrot(self.tick + 0.5*self.period,self.period)            
        
            self.sendData()
            
            r.sleep()
        
        return "Transform"
    
    
    def getPosTrot(self,timestamp,period):
        # this thing is written like shit
        
        PI = 3.1415926
        leaveGroundAngleThres = - PI/2.5
        leaveGrooundTimeThres = 0.8
        
        x = (timestamp % period)/float(period)
        turns = int(timestamp/period)
        y = 0
        k1 = (leaveGroundAngleThres - 0)/(leaveGrooundTimeThres)
        k2 = (- PI - leaveGroundAngleThres)/(1-leaveGrooundTimeThres)
        
        if x < leaveGrooundTimeThres:
            y = k1*x
        else:
            y = leaveGroundAngleThres + k2*(x-leaveGrooundTimeThres)
        
        y = (y - PI/2.0 + PI + turns * PI) % (2 * PI) - PI
        
        return y