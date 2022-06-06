#!/usr/bin/env python
import rospy
import smach
from States.Motor import Motor, MotorManager
from wheelleg_hexapod_control.msg import WheelLegControlMsg
from sensor_msgs.msg import Joy

class ContorlMode:
    POS_MODE = 1
    SPD_MODE = 0

class RobotState(smach.State):
    
    def __init__(self,outcomes):
        
        smach.State.__init__(self, outcomes=outcomes)
        
        self.motorNameList = ["RM_Joint","RF_Joint","LF_Joint","LM_Joint","LB_Joint","RB_Joint"]
        
        for name in self.motorNameList:
            Motor(name)
        
        self.motorControlMode = ContorlMode.SPD_MODE
        self.IsLeggedMode = False
        
        self.jointPub = rospy.Publisher('/WheelLegHexapod/command',WheelLegControlMsg,queue_size=10)
        self.joySub   = rospy.Subscriber('joy',Joy,self.JoystickCallback)
        self.joyData  = None
        
        self.stateChangeFlag = False
    
    def execute(self, userdata):
        ROS_INFO("'execute' method must be overridden")


    def sendData(self): # send joint commands
        msg = WheelLegControlMsg()
        msg.IsLeggedMode = self.IsLeggedMode            
        for motorName in self.motorNameList:
            motor = MotorManager.instance().getMotor(motorName) # get motor from motormanager
            msg.JointName.append(motorName)
            msg.JointMode.append(int(self.motorControlMode))
            if self.motorControlMode == ContorlMode.SPD_MODE:
                msg.JointData.append(motor.speedSet)
            else:
                msg.JointData.append(motor.positionSet)
        self.jointPub.publish(msg)
    
    def JoystickCallback(self,data):
        if(data.buttons[1]==1 and 
        (self.joyData is None or self.joyData.buttons[1]!=1) # compare current joystick data with previous data
        ): 
            self.stateChangeFlag = True # Change state when B is pressed
        self.joyData = data # collect joystick data