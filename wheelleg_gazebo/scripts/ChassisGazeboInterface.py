#!/usr/bin/env python
import rospy
from JointController import TransformJointController , WheelJointController, JointControllerManager
from controller_manager_msgs.srv import SwitchController
from wheelleg_control.msg import WheelLegControlMsg


def jointCommandCallback(msg):
    jointCommandCallback.msg = msg
    print(msg)

if __name__ == '__main__':    

    rospy.init_node('chassisController', anonymous=True)
    r = rospy.Rate(100)
    
    robotName = "WheelLegHexapod"
    
    #wait for switch_controller
    rospy.wait_for_service('/' + robotName + '/controller_manager/switch_controller')
    switchServiceProxy = rospy.ServiceProxy('/' + robotName + '/controller_manager/switch_controller', SwitchController)
    
    #initialize command callback
    jointCommandCallback.msg = None
    commandSub = rospy.Subscriber('/' + robotName + '/command',WheelLegControlMsg,jointCommandCallback)
    
    #Initialize gazebo Joint controllers
    wheelJointNameList = rospy.get_param('/' + robotName + '/joints')
    wheelJointControllerDict = {}
    for name in wheelJointNameList:
        joint_name = name + "_Joint"
        wheelJointControllerDict[joint_name] = WheelJointController(name=joint_name,robotName=robotName,mode=0)
        
    #Initialize Transform Controller
    legWheelController = TransformJointController(robotName=robotName)
    
    while not rospy.is_shutdown():

        if jointCommandCallback.msg is not None:
            
            for i in range(len(jointCommandCallback.msg.JointName)):
                
                # get corresponding joint
                jointname = jointCommandCallback.msg.JointName[i]
                joint = wheelJointControllerDict[jointname]
                
                # set joint mode
                joint.setMode(jointCommandCallback.msg.JointMode[i])
                
                # set joint command
                if joint.mode == 0: #vel
                    joint.speedSet = jointCommandCallback.msg.JointData[i]
                else: #pos
                    joint.positionSet = jointCommandCallback.msg.JointData[i]
            
            # update transform joint
            legWheelController.isLegged = jointCommandCallback.msg.IsLeggedMode 
            
        # send command to all gazebo controllers
        JointControllerManager.instance().SendCommand()
        
        r.sleep()
        
