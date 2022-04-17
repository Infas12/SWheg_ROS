#!/usr/bin/env python
import rospy
from JointController import TransformJointController , WheelJointController, JointControllerManager
from controller_manager_msgs.srv import SwitchController
from wheelleg_control.msg import JointData


def jointCommandCallback(msg):
    jointCommandCallback.msg = msg
    print(msg)


if __name__ == '__main__':    
    

    rospy.init_node('chassisController', anonymous=True)
    r = rospy.Rate(100)
    
    #wait for switch_controller
    rospy.wait_for_service('/WheelLeg/controller_manager/switch_controller')
    switchServiceProxy = rospy.ServiceProxy('/WheelLeg/controller_manager/switch_controller', SwitchController)
    
    #initialize command callback
    jointCommandCallback.msg = None
    commandSub = rospy.Subscriber('/WheelLeg/command',JointData,jointCommandCallback)
    
    #initialize joint callback

    #Initialize Joint controllers
    wheelJointNameList = ["LF","LM","LB","RF","RM","RB"]
    wheelJointControllerList = []
    for name in wheelJointNameList:
        wheelJointControllerList.append(WheelJointController(name=name,mode=0))
    legWheelController = TransformJointController.instance()
    
    
    while not rospy.is_shutdown():

        if jointCommandCallback.msg is not None:
            # update wheel joints
            for i in range(len(wheelJointNameList)):
                wheelJointControllerList[i].setMode(jointCommandCallback.msg.JointMode[i])
                installation_coef = -1
                if i < 3: # Right sided joints have a reversed installation direction
                    installation_coef = 1
                if wheelJointControllerList[i].mode == 0: # vel
                    wheelJointControllerList[i].speedSet = jointCommandCallback.msg.JointData[i] * installation_coef
                elif wheelJointControllerList[i].mode == 1: # pos
                    wheelJointControllerList[i].pisitionSet = jointCommandCallback.msg.JointData[i] * installation_coef

            # update transform joint
            if jointCommandCallback.msg.TransformLength > 1.0:  #For now we only have two states
                legWheelController.isLegged = True
            else:
                legWheelController.isLegged = False
            

        JointControllerManager.instance().SendCommand()
        r.sleep()
        