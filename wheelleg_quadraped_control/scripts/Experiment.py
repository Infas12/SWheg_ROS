#!/usr/bin/env python
import rospy
from States.Motor import Motor, MotorManager
from States.RobotState import RobotState, ContorlMode
from sensor_msgs.msg import Joy
import numpy as np
import sys

class DataTest:
    def __init__(self):
        self.motorNameList = ["LF_Joint","LB_Joint","RB_Joint","RF_Joint"]
        for name in self.motorNameList:
            Motor(name)

        self.joySub   = rospy.Subscriber('joy',Joy,self.JoystickCallback)
        self.joyData  = None

        self.dataFinish = True
        self.powerFile = 0

        self.startTime = 0
        self.endTime = 0
        

    def JoystickCallback(self,data):
        if(data.buttons[0]==1 and 
        (self.joyData is None or self.joyData.buttons[0]!=1) # compare current joystick data with previous data
        ): 
            if(self.dataFinish):
                rospy.logwarn("write start")
                self.powerFile = open("four_motor_power.txt","w")
                self.dataFinish = False
                self.startTime = rospy.get_rostime()
            else:
                self.dataFinish = True
                rospy.logwarn("write finished")
                self.endTime = rospy.get_rostime()
                startString = "" + str(self.startTime.secs) + "  " + str(self.startTime.nsecs) + "\n"
                endString = "" + str(self.endTime.secs) + "  " + str(self.endTime.nsecs) + "\n"
                self.powerFile.write(startString)
                self.powerFile.write(endString)
                self.powerFile.close()
        self.joyData = data # collect joystick data



    def write_power(self):
        rospy.init_node('experiment')
        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            if(not self.dataFinish):
                currentList = []
                for motorName in self.motorNameList:
                    currentList.append(MotorManager.instance().getMotor(motorName).currentFdb)
                strList = [str(i) for i in currentList]
                myString = " ".join(strList)
                myString = myString + "\n"
                self.powerFile.write(myString)
            r.sleep()

        rospy.spin()


if __name__ == '__main__':
    dataTest = DataTest()
    dataTest.write_power()