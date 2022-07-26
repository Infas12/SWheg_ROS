#!/usr/bin/env python
import rospy
import smach
from States.Motor import Motor, MotorManager
from wheelleg_control.msg import WheelLegControlMsg
from sensor_msgs.msg import Joy

class ContorlMode:
    POS_MODE = 1
    SPD_MODE = 0

class RobotState(smach.State):
    
    def __init__(self,outcomes):
        
        smach.State.__init__(self, outcomes=outcomes)
        
        self.motorNameList = ["LF_Joint","LM_Joint","LB_Joint","RB_Joint","RM_Joint","RF_Joint"]
        
        for name in self.motorNameList:
            Motor(name)
        
        self.motorControlMode = ContorlMode.SPD_MODE
        self.IsLeggedMode = False
        
        self.jointPub = rospy.Publisher('/WheelLegHexapod/command',WheelLegControlMsg,queue_size=10)
        self.joySub   = rospy.Subscriber('joy',Joy,self.JoystickCallback)
        self.joyData  = None
        
        self.Apressed = False
        self.Bpressed = False
        self.Xpressed = False
        self.Ypressed = False
        
    
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
        
        if(data.buttons[0]==1 and 
        (self.joyData is None or self.joyData.buttons[0]!=1) 
        ): 
            self.Apressed = True 
        
        if(data.buttons[1]==1 and 
        (self.joyData is None or self.joyData.buttons[1]!=1) # compare current joystick data with previous data
        ): 
            self.Bpressed = True
        
        if(data.buttons[2]==1 and 
        (self.joyData is None or self.joyData.buttons[2]!=1) # compare current joystick data with previous data
        ): 
            self.Xpressed = True
            
        if(data.buttons[3]==1 and 
        (self.joyData is None or self.joyData.buttons[3]!=1) # compare current joystick data with previous data
        ): 
            self.Ypressed = True           
            
        self.joyData = data # collect joystick data