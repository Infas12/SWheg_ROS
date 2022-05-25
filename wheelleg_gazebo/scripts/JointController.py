#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from Singleton import Singleton
from controller_manager_msgs.srv import SwitchController

@Singleton
class JointControllerManager:
    def __init__(self):
        self.controllerList = []
    
    def SendCommand(self):
        for controller in self.controllerList:
            controller.SendCommand()

@Singleton
class TransformJointController:
    def __init__(self):
        self.name_list = ["LFRA","LMRA","LBRA","RFRA","RMRA","RBRA",
                 "LFRB","LMRB","LBRB","RFRB","RMRB","RBRB"]
        self.transformer_joint_publisher_list = []
        
        for name in self.name_list:
            self.transformer_joint_publisher_list.append(
                rospy.Publisher('/WheelLeg/' + name +'_Joint_position_controller/command',Float64,queue_size=10)
            )
        
        self.isLegged = False
        self.joint_angle = 0.0
        JointControllerManager.instance().controllerList.append(self)
        
    def SendCommand(self):
        
        if self.isLegged :
            self.joint_angle = -0.5
        else :
            self.joint_angle = -0.0
        
        for pub in self.transformer_joint_publisher_list:
            pub.publish(self.joint_angle)

class WheelJointController:
    
    def __init__(self,name,mode = 0):
        
        self.positionController = name + '_position_controller'
        self.velocityController = name + '_velocity_controller'
        
        self.speedSet = 0.0
        self.positionSet = 0.0
        self.mode = mode #0 - speed mode 1 - vel mode
        
        self.speedCmdPub = rospy.Publisher(
            '/WheelLeg/' + self.velocityController +'/command',
            Float64,queue_size=10)
        
        self.positionCmdPub = rospy.Publisher(
            '/WheelLeg/' + self.positionController +'/command',
            Float64,queue_size=10)
        
        self.switchServiceProxy = rospy.ServiceProxy(
            '/WheelLeg/controller_manager/switch_controller', SwitchController)
        
        JointControllerManager.instance().controllerList.append(self)
    
    def setMode(self,mode):
        
        if self.mode == mode:
            return
        else:
            self.mode = mode
        
        if mode == 0: # position to velocity
            self.switchServiceProxy([self.velocityController],[self.positionController],2,1,1)
        else:         # position to position
            self.switchServiceProxy([self.positionController],[self.velocityController],2,1,1)
            
    def SendCommand(self):
        
        if self.mode == 0: #speed mode
            self.speedCmdPub.publish(self.speedSet)
        else:
            self.positionCmdPub.publish(self.positionSet)
        