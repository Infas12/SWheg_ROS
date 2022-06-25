#!/usr/bin/env python
import rospy
from States.Motor import MotorManager
from States.RobotState import RobotState, ContorlMode


class TestState(RobotState):
    
    def __init__(self):
        
        RobotState.__init__(self, outcomes=["Finished"])
        
        self.motorControlMode = ContorlMode.SPD_MODE # In transition state we'll control position
        
        self.IsLeggedMode = False # remain wheel mode
        
        self.tramsformCompleted = False # transform complete flag
        
        self.tick = 0 # timer
        
    def execute(self, userdata):
        r = rospy.Rate(1000)
        
        # clean-up all data
        self.tick = 0

        while(self.tick<10000):
            
            self.tick += 1

            if self.tick%1000 == 0:
                now = rospy.get_rostime()
                rospy.logwarn('tick: %f, Current time %i %i',self.tick,now.secs, now.nsecs)

            if self.tick < 1000:
                MotorManager.instance().getMotor("LF_Joint").speedSet = 5.0
                MotorManager.instance().getMotor("LB_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("RF_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("RB_Joint").speedSet = 0.0
            elif self.tick < 2000:
                MotorManager.instance().getMotor("LF_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("LB_Joint").speedSet = 2.0
                MotorManager.instance().getMotor("RF_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("RB_Joint").speedSet = 0.0
            elif self.tick < 3000:
                MotorManager.instance().getMotor("LF_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("LB_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("RF_Joint").speedSet = 2.0
                MotorManager.instance().getMotor("RB_Joint").speedSet = 0.0
            elif self.tick < 4000:
                MotorManager.instance().getMotor("LF_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("LB_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("RF_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("RB_Joint").speedSet = 2.0       
            elif self.tick < 5000:
                MotorManager.instance().getMotor("LF_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("LB_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("RF_Joint").speedSet = 0.0
                MotorManager.instance().getMotor("RB_Joint").speedSet = 0.0
                self.motorControlMode = ContorlMode.POS_MODE
            elif self.tick < 6000:
                MotorManager.instance().getMotor("LF_Joint").positionSet = 0.5
            elif self.tick < 7000:
                MotorManager.instance().getMotor("LB_Joint").positionSet = 0.5
            elif self.tick < 8000:
                MotorManager.instance().getMotor("RF_Joint").positionSet = 0.5
            elif self.tick < 9000:
                MotorManager.instance().getMotor("RB_Joint").positionSet = 0.5
            
            
            self.sendData()
            r.sleep()
        
        return "TransformCompleted"
    