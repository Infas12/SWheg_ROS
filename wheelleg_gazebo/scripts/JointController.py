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

class TransformJointController:
    def __init__(self,robotName):
        
        # get wheel joints
        wheelJointNameList = rospy.get_param('/' + robotName + '/joints')
        
        # get rim joint name
        self.rim_joint_name_list = []
        for joint in wheelJointNameList:
            self.rim_joint_name_list.append(joint+"RA")
            self.rim_joint_name_list.append(joint+"RB")
        
        # spawn rim joint controller publishers
        self.transformer_joint_publisher_list = []
        for name in self.rim_joint_name_list:
            self.transformer_joint_publisher_list.append(
                rospy.Publisher('/' + robotName + '/' + name +'_Joint_position_controller/command',
                                Float64,
                                queue_size=10)
            )
        
        # set joint mode
        self.isLegged = False
        self.joint_angle = 0.0
        
        # register at joint controller manager
        JointControllerManager.instance().controllerList.append(self)
        
    def SendCommand(self):
        
        if self.isLegged :
            self.joint_angle = -0.5
        else :
            self.joint_angle = -0.0
        
        for pub in self.transformer_joint_publisher_list:
            pub.publish(self.joint_angle)

class WheelJointController:
    
    def __init__(self,name,robotName,mode = 0):
        
        self.positionController = name + '_position_controller'
        self.velocityController = name + '_velocity_controller'
        
        self.speedSet = 0.0
        self.positionSet = 0.0
        self.mode = mode #0 - speed mode 1 - vel mode
        
        self.speedCmdPub = rospy.Publisher(
            '/' + robotName + '/' + self.velocityController +'/command',
            Float64,queue_size=10)
        
        self.positionCmdPub = rospy.Publisher(
            '/' + robotName + '/' + self.positionController +'/command',
            Float64,queue_size=10)
        
        self.switchServiceProxy = rospy.ServiceProxy(
            '/' + robotName + '/controller_manager/switch_controller',
            SwitchController)
        
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
        