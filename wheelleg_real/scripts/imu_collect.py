#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
import numpy as np
import sys

class DataTest:
    def __init__(self):
        self.joySub = rospy.Subscriber('joy',Joy,self.JoystickCallback)
        self.imuSub = rospy.Subscriber("wit/imu", Imu,self.ImuDataCallback)
        self.joyData = None
        self.imuData = None

        self.dataFinish = True
        self.isImuNew = False
        self.powerFile = 0

        self.startTime = 0
        self.endTime = 0
        

    def JoystickCallback(self,data):
        if(data.buttons[0]==1 and 
        (self.joyData is None or self.joyData.buttons[0]!=1) # compare current joystick data with previous data
        ): 
            if(self.dataFinish):
                rospy.logwarn("write start")
                self.powerFile = open("accel.txt","w")
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

    def ImuDataCallback(self,data):
        self.imuData = data 
        self.isImuNew = True


    def write_accel(self):
        rospy.init_node('IMU_collection')
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            if((not self.dataFinish) and self.isImuNew):
                accelVec = self.imuData.linear_acceleration
                myString = str(accelVec.x) + " "+ str(accelVec.y) + " " + str(accelVec.z) + "\n"
                self.powerFile.write(myString)
                self.isImuNew = False
            r.sleep()
        rospy.spin()


if __name__ == '__main__':
    dataTest = DataTest()
    dataTest.write_accel()