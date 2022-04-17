#!/usr/bin/env python
import imp
from operator import mod
import rospy
from sensor_msgs.msg import Joy
from JointController import TransformJointController , WheelJointController, JointControllerManager
from std_msgs.msg import Float64
from controller_manager_msgs.srv import SwitchController
from wheelleg_control.msg import JointState


def JoystickCallback(data):
    
    if(data.buttons[1]==1 and 
       (JoystickCallback.prevdata is None or JoystickCallback.prevdata.buttons[1]!=1)
       ):
        JoystickCallback.isLegged = not JoystickCallback.isLegged
        
    JoystickCallback.prevdata = data



if __name__ == '__main__':    
    

    rospy.init_node('chassisController', anonymous=True)
    r = rospy.Rate(100)
    legWheelController = TransformJointController.instance()
    
    #wait for switch controller
    rospy.wait_for_service('/WheelLeg/controller_manager/switch_controller')
    switchServiceProxy = rospy.ServiceProxy('/WheelLeg/controller_manager/switch_controller', SwitchController)
    
    #test stuff
    #initialize callback
    JoystickCallback.isLegged = False
    JoystickCallback.prevdata = None
    joySub = rospy.Subscriber('joy',Joy,JoystickCallback)


    Vy = 0.0
    Vw = 0.0

    left = ["LF","LM","LB"]
    right = ["RF","RM","RB"]
    global left_joint, right_joint
    left_joint  = []
    right_joint = []
    for name in left:
        left_joint.append(
            WheelJointController(name=name,mode=0)
        )        
    for name in right:
        right_joint.append(
            WheelJointController(name=name,mode=0)
        )       
    
    
    while not rospy.is_shutdown():
        
        # update joystick readings
        if(JoystickCallback.prevdata is not None):
            legWheelController.isLegged = JoystickCallback.isLegged
            Vy = 25.0 * JoystickCallback.prevdata.axes[0]
            Vw = -25.0 * JoystickCallback.prevdata.axes[1]

        if JoystickCallback.isLegged:
            for joint in left_joint:
                joint.setMode(1)
            for joint in right_joint:
                joint.setMode(1)
        else:
            for joint in left_joint:
                joint.setMode(0)
            for joint in right_joint:
                joint.setMode(0)            

        # control joints
        for joint in left_joint:
            joint.speedSet = Vy + Vw
        for joint in right_joint:
            joint.speedSet = Vy - Vw
        

        JointControllerManager.instance().SendCommand()
        
        
        r.sleep()
        