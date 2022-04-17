#!/usr/bin/env python
import rospy
from JointController import TransformJointController , WheelJointController, JointControllerManager
from controller_manager_msgs.srv import SwitchController
from wheelleg_control.msg import JointState


def jointCommandCallback(msg):
    jointCommandCallback.msg = msg
    print(msg)


if __name__ == '__main__':    
    

    rospy.init_node('chassisController', anonymous=True)
    r = rospy.Rate(100)
    
    
    #wait for switch controller
    rospy.wait_for_service('/WheelLeg/controller_manager/switch_controller')
    switchServiceProxy = rospy.ServiceProxy('/WheelLeg/controller_manager/switch_controller', SwitchController)
    
    #initialize callback
    jointCommandCallback.msg = None
    commandSub = rospy.Subscriber('/WheelLeg/command',JointState,jointCommandCallback)


    #Initialize Joint controller
    jointNameList = ["LF","LM","LB","RF","RM","RB"]
    jointControllerList = []
    for name in jointNameList:
        jointControllerList.append(WheelJointController(name=name,mode=0))
    legWheelController = TransformJointController.instance()
    
    while not rospy.is_shutdown():

        if jointCommandCallback.msg is not None:
            # update joystick readings
            for i in range(len(jointNameList)):
                jointControllerList[i].mode = jointCommandCallback.msg.JointMode[i]
                if jointControllerList[i].mode == 0: # vel
                    jointControllerList[i].speedSet = jointCommandCallback.msg.JointData[i]
        

        JointControllerManager.instance().SendCommand()
        r.sleep()
        