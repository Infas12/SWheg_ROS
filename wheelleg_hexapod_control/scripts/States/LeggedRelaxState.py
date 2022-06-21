#!/usr/bin/env python
import rospy
from States.Motor import MotorManager
from RobotState import RobotState, ContorlMode


class LeggedRelaxState(RobotState):
    
    def __init__(self):
        
        RobotState.__init__(self, outcomes=["Walk","Wheel"])
        
        self.motorControlMode = ContorlMode.POS_MODE 
        
        self.IsLeggedMode = True 
                
        self.initialPos = {} # initial pos of motors when entering the state
        
        self.targetPos = {"LF_Joint":0.0,
                          "LM_Joint":0.0,
                          "LB_Joint":0.0,
                          "RB_Joint":0.0,
                          "RM_Joint":0.0,
                          "RF_Joint":0.0
                          }
        
        self.changePos = {}
        self.tramsformCompleted = False # transform complete flag
        self.tick = 0 # timer
        self.transformDuration = 1000 # the duration of transform in ms
        
    def execute(self, userdata):
        r = rospy.Rate(1000)
        
        for name in self.motorNameList:
            self.initialPos[name] = MotorManager.instance().getMotor(name).positionFdb # Get the initial position of the motor
            self.changePos[name] = (self.targetPos[name] - self.initialPos[name]) % (2.0*3.14159)  # calculate the change of angle of this motor ([-pi to pi])
            if(self.changePos[name]>3.14159):
                self.changePos[name] = self.changePos[name] - 2.0*3.14159
        # clean-up all data
        self.tramsformCompleted = False
        self.stateChangeFlag = False
        self.tick = 0
        
        err =  [abs(ele) for ele in self.changePos.values()]
        if sum(err)>0.1:            
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
    
        while self.joyData.axes[1] == 0 and not self.stateChangeFlag:
            self.sendData()
            r.sleep()
        
        if self.stateChangeFlag:
            return "Wheel"
        else:
            return "Walk"
    