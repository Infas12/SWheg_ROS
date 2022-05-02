#!/usr/bin/env python
import rospy
from States.Motor import MotorManager
from RobotState import RobotState, ContorlMode


class TransitionState(RobotState):
    
    def __init__(self):
        RobotState.__init__(self, outcomes=["TransformCompleted"])
        self.motorControlMode = ContorlMode.POS_MODE
        self.initialPos = {}
        self.targetPos = {"LF_Joint":0.0,"LM_Joint":3.14159/2.0,"LB_Joint":0.0,"RB_Joint":3.14159/2.0,"RM_Joint":0.0,"RF_Joint":3.14159/2.0}
        self.changePos = {}
        self.transformLength = 0.0
        self.tramsformCompleted = False
        self.tick = 0
        self.transformDuration = 1000
        
    def execute(self, userdata):
        r = rospy.Rate(1000)
        
        for name in self.motorNameList:
            self.initialPos[name] = MotorManager.instance().getMotor(name).positionFdb
            self.changePos[name] = (self.targetPos[name] - self.initialPos[name]) % (2.0*3.14159) - 3.14159

        # clean-up all data
        self.tramsformCompleted = False #gonna fix this.
        self.tick = 0

        while(self.tick<self.transformDuration):
            
            self.tick += 1
            
            for name in self.motorNameList:
                motor = MotorManager.instance().getMotor(name)
                alpha = self.tick/float(self.transformDuration)
                motor.positionSet = self.initialPos[name] + alpha*self.changePos[name]
                
            self.sendData()
            r.sleep()
        
        return "TransformCompleted"
    