#!/usr/bin/env python
import imp
import rospy
from ChassisController import ChassisController

if __name__ == '__main__':
    rospy.init_node('chassisController', anonymous=True)
    r = rospy.Rate(100)
    testController = ChassisController()
    
    
    while not rospy.is_shutdown():

        testController.update()
        r.sleep()

