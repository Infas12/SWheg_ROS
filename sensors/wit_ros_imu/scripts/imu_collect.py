#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
import numpy as np
import sys
import rosbag
import time

class DataTest:
    def __init__(self):
        self.joySub = rospy.Subscriber('joy',Joy,self.JoystickCallback)
        self.imuSub = rospy.Subscriber("wit/imu", Imu,self.ImuDataCallback)
        self.joyData = None
        self.imuData = None

        self.dataFinish = True
        self.isImuNew = False
        self.bag = 0

    def JoystickCallback(self,data):
        if(data.buttons[8]==1 and 
        (self.joyData is None or self.joyData.buttons[8]!=1) # power button
        ): 
            if(self.dataFinish):
                rospy.logwarn("write start")
                recordname = time.strftime("%Y%m%d%H%M%S", time.localtime()) + '.bag'
                self.bag = rosbag.Bag(recordname, 'w')
                self.dataFinish = False
            else:
                self.dataFinish = True
                rospy.logwarn("write finished")
                self.bag.close()
        self.joyData = data # collect joystick data

    def ImuDataCallback(self,data):
        self.imuData = data 
        self.isImuNew = True


    def write_accel(self):
        rospy.init_node('IMU_collection')
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            if((not self.dataFinish) and self.isImuNew):
                self.bag.write('wit/imu',self.imuData)
                self.isImuNew = False
            r.sleep()
        rospy.spin()


if __name__ == '__main__':
    dataTest = DataTest()
    dataTest.write_accel()