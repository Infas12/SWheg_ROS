#!/usr/bin/env python
import rospy
from States.Motor import MotorManager
from RobotState import RobotState, ContorlMode


class TransitionState(RobotState):
    
    def __init__(self):
        
        RobotState.__init__(self, outcomes=["TransformCompleted"])
        
        self.motorControlMode = ContorlMode.POS_MODE # In transition state we'll control position
        
        self.IsLeggedMode = False # remain wheel mode
                
        self.initialPos = {} # initial pos of motors when entering the state
        
        self.targetPos = {"L_Joint":0.0,
                          "R_Joint":0.0
                          }
        
        self.changePos = {}
        
        self.tramsformCompleted = False # transform complete flag
        
        self.tick = 0 # timer
        
        self.transformDuration = 1000 # the duration of transform in ms
        
    def execute(self, userdata):
        r = rospy.Rate(1000)
        
        
        for name in self.motorNameList:
            self.initialPos[name] = MotorManager.instance().getMotor(name).positionFdb # Get the initial position of the motor
            self.changePos[name] = (self.targetPos[name] - self.initialPos[name]) % (2.0*3.14159) - 3.14159 # calculate the change of angle of this motor ([-pi to pi])

        # clean-up all data
        self.tramsformCompleted = False 
        self.tick = 0

        while(self.tick<self.transformDuration):
            
            self.tick += 1
            
            for name in self.motorNameList:
                # linear interp of motor position
                # theta = theta_0 + (1-alpha)*theta_target
                motor = MotorManager.instance().getMotor(name)
                alpha = self.tick/float(self.transformDuration)
                motor.positionSet = self.initialPos[name] + alpha*self.changePos[name]
                
            self.sendData()
            r.sleep()
        
        return "TransformCompleted"
    