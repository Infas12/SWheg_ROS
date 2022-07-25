#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from States.Singleton import Singleton
# from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState

@Singleton
class MotorManager:
    
    def __init__(self):
        self.motorDict = {}
        self.jointFdbSub = rospy.Subscriber('/WheelLegHexapod/joint_states',JointState,self.JointStateCallback)
    
    def JointStateCallback(self,data): # collect and distribute all motor feedback
        posFdbDict = dict(zip(data.name,data.position))
        spdFdbDict = dict(zip(data.name,data.velocity))
        curFdbDict = dict(zip(data.name,data.effort))
        for controller_name in self.motorDict.keys(): # set motor feedback
            self.motorDict[controller_name].speedFdb    = spdFdbDict[controller_name]
            self.motorDict[controller_name].positionFdb = posFdbDict[controller_name]
            self.motorDict[controller_name].currentFdb = curFdbDict[controller_name]
    
    def getMotor(self,motorName): #get motor instance
        return self.motorDict[motorName]

class Motor:
    
    def __init__(self,name):
        self.speedSet = 0.0
        self.speedFdb = 0.0
        self.positionSet = 0.0
        self.positionFdb = 0.0
        self.currentSet = 0.0
        self.currentFdb = 0.0
        self.name = name
        MotorManager.instance().motorDict[name] = self #register motor at manager
    
        
        