#!/usr/bin/env python
import rospy
from States.Motor import MotorManager
from RobotState import RobotState, ContorlMode


class WheelState(RobotState):
    
    def __init__(self):
        RobotState.__init__(self, outcomes=["Transform"])
        self.motorControlMode = ContorlMode.SPD_MODE
        self.Mode = 0 #wheel mode
        self.Vy = 0.0
        self.Vw = 0.0
        
        
    def execute(self, userdata):
        r = rospy.Rate(100)
        
        # clean-up all data
        self.stateChangeFlag = False #gonna fix this.
        self.Vy = 0.0
        self.Vw = 0.0
        
        print(self.Vy)

        while(not self.stateChangeFlag):

            if self.joyData is not None:
                self.Vw = 25.0 * self.joyData.axes[0]
                self.Vy = 25.0 * self.joyData.axes[1]
            
            MotorManager.instance().getMotor("LF_Joint").speedSet = (self.Vy-self.Vw)*1.0
            MotorManager.instance().getMotor("LB_Joint").speedSet = (self.Vy-self.Vw)*1.0
            MotorManager.instance().getMotor("RF_Joint").speedSet = (self.Vy+self.Vw)*-1.0
            MotorManager.instance().getMotor("RB_Joint").speedSet = (self.Vy+self.Vw)*-1.0                        
        
            self.sendData()
            
            r.sleep()
        
        return "Transform"
    