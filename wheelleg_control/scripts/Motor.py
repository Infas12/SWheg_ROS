#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from Singleton import Singleton
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState

@Singleton
class MotorManager:
    
    def __init__(self):
        self.motorDict = {}
        self.jointFdbSub = rospy.Subscriber('/WheelLeg/joint_states',JointState,self.JointStateCallback)
    
    def JointStateCallback(self,data):
        posFdbDict = dict(zip(data.name,data.position))
        spdFdbDict = dict(zip(data.name,data.velocity))
        for controller_name in self.motorDict.keys():
            self.motorDict[controller_name].speedFdb    = spdFdbDict[controller_name]
            self.motorDict[controller_name].positionFdb = posFdbDict[controller_name]
    
    def getMotor(self,motorName):
        return self.motorDict[motorName]


class Motor:
    
    def __init__(self,name):
        
        self.speedSet = 0.0
        self.positionSet = 0.0
        self.speedFdb = 0.0
        self.positionFdb = 0.0
        self.name = name

        MotorManager.instance().controllerDict[name] = self
    
        
        