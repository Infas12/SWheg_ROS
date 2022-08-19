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
        self.sub = rospy.Subscriber("/wit/imu", Imu, self.imuMsgCallback)
        self.pub = rospy.Publisher("/imu",Imu)
        self.f = open(os.getcwd()+"/QuadrupedStair.csv",'w')
        self.writer = csv.writer(self.f)
        self.initialized = False
        
    def imuMsgCallback(self,msg):
        
        # calculate the correct orientation of imu.
        q_imu = quaternion_from_euler(msg.orientation.x/180.0*3.14,msg.orientation.y/180.0*3.14,msg.orientation.z/180.0*3.14)
        msg.orientation.x = q_imu[0]
        msg.orientation.y = q_imu[1]
        msg.orientation.z = q_imu[2]
        msg.orientation.w = q_imu[3]
        self.pub.publish(msg) # send the correct transform
        
        # calculate the orientation of robot        
        imu_rotation_matrix = quaternion_matrix(q_imu)
        
        # imu_installation_matrix = numpy.array([[1, 0, 0, 0],
        #                                        [0,-1, 0, 0],
        #                                        [0, 0,-1, 0],
        #                                        [0, 0, 0, 1]],dtype=numpy.float64) #for hexapod tests.
        
        imu_installation_matrix = numpy.array([[ 0,-1, 0, 0],
                                               [ 1, 0, 0, 0],
                                               [ 0, 0, 1, 0],
                                               [ 0, 0, 0, 1]],dtype=numpy.float64) #for quadruped wheel and stair tests.        
        
        body_rotation_matrix =  numpy.dot(imu_rotation_matrix,imu_installation_matrix.T)
        q_body = quaternion_from_matrix(body_rotation_matrix)
        euler = euler_from_matrix(body_rotation_matrix,axes='szyx') # rotation sequence yaw-pitch-roll.
        
        # send transform for visualization
        br = tf.TransformBroadcaster()
        br.sendTransform((0,0,0),
                         q_body,
                         rospy.Time.now(),
                         "body_frame",
                         "world")
        # br.sendTransform((0,0,0),
        #             q_imu,
        #             rospy.Time.now(),
        #             "imu_frame",
        #             "world")
               
        # cost calculation
        cost = 1000 * (euler[1] ** 2 + euler[2] ** 2) # roll^2+yaw^2
        self.writer.writerow([cost,
                              euler[0],
                              euler[1],
                              euler[2],
                              msg.orientation.x/180.0*3.14,
                              msg.orientation.y/180.0*3.14,
                              msg.orientation.z/180.0*3.14])
        
        
        # self.writer.writerow([abs(cost[3]), cost[0], cost[1], cost[2], cost[3],msg.orientation.x,msg.orientation.y,msg.orientation.z])
    
    def __del__(self):
        self.f.close()

if __name__ == "__main__":

    rospy.init_node("smoothnessCalculator")
    r = rospy.Rate(100)
    testlogger = ImuMsgHandler()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        del testlogger