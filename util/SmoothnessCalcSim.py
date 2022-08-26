#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import *
import csv
import os
import numpy 
import tf

class ImuMsgHandler:
    
    def __init__(self):
        self.sub = rospy.Subscriber("/imu/data", Imu, self.imuMsgCallback)
        self.f = open(os.getcwd()+"/LegTripod.csv",'w')
        self.writer = csv.writer(self.f)
        self.writer.writerow(["cost",
                        "yaw",
                        "pitch",
                        "roll"])
        self.initialized = False
        self.cnt = 0
        
        
    def imuMsgCallback(self,msg):
        
        if self.cnt < 120:
            self.cnt += 1
        else:
            raise BaseException("Done")
        
        q_imu = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w] 
        imu_rotation_matrix = quaternion_matrix(q_imu)
        euler = euler_from_matrix(imu_rotation_matrix,axes='szyx') # rotation sequence yaw-pitch-roll.

        # cost calculation
        cost = 1000 * (euler[1] ** 2 + euler[2] ** 2) # roll^2+yaw^2
        self.writer.writerow([cost,
                              euler[0],
                              euler[1],
                              euler[2]])
    
    def __del__(self):
        self.f.close()

if __name__ == "__main__":

    rospy.init_node("smoothnessCalculator")
    r = rospy.Rate(100)
    testlogger = ImuMsgHandler()
    try:
        rospy.spin()
    except BaseException:
        del testlogger