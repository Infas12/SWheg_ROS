#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from servo import servo
from scservo_sdk import *
import numpy as np
import sys
target_speed1 = 0 # 32768
target_speed2 = 0 # 32768
target_speed3 = 0 # 32768
speedTag = 400

def JoystickCallback(data):
    global speedTag, target_speed1, target_speed2, target_speed3

    if(data.axes[2]<0): #LT
        if(data.buttons[4]==1):#LB 打开
            target_speed1 = 32768 + speedTag
            target_speed2 = speedTag
            target_speed3 = speedTag
        elif(data.buttons[5]==1):#RB 闭合
            target_speed1 = speedTag
            target_speed2 = 32768 + speedTag
            target_speed3 = 32768 + speedTag
        elif(data.buttons[7]==1):#start 中间轮打开
            target_speed1 = 32768 + speedTag
            target_speed2 = target_speed3 = 0
        elif(data.buttons[6]==1):#back 中间轮闭合
            target_speed1 = speedTag
            target_speed2 = target_speed3 = 0
        elif(data.axes[6]==1):#lr 后轮
            target_speed1 = target_speed2 = 0
            target_speed3 = speedTag
        elif(data.axes[6]==-1):#lr 后轮
            target_speed1 = target_speed2 = 0
            target_speed3 = 32768 + speedTag
        elif(data.axes[7]==1):#ud 前轮
            target_speed1 = target_speed3 = 0
            target_speed2 = speedTag
        elif(data.axes[7]==-1):#ud 前轮
            target_speed1 = target_speed3 = 0
            target_speed2 = 32768 + speedTag
        else:
            target_speed1 = target_speed2 = target_speed3 = 0
    else:
        target_speed1 = target_speed2 = target_speed3 = 0

        

def servo_control():
    global target_speed

    # Serial Setup 注意检查是否配置正确
    protocol_end                = 0           # SCServo bit end(STS/SMS=0, SCS=1)

    BAUDRATE                    = 115200  
    DEVICENAME                  = '/dev/ttyUSB1'   
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(protocol_end)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    servo1 = servo(1,packetHandler,portHandler)
    servo2 = servo(2,packetHandler,portHandler)

    DEVICENAME2                 = '/dev/ttyUSB2'   
    portHandler2 = PortHandler(DEVICENAME2)
    packetHandler2 = PacketHandler(protocol_end)
    portHandler2.openPort()
    portHandler2.setBaudRate(BAUDRATE)
    servo3 = servo(3,packetHandler2,portHandler2)

    sub = rospy.Subscriber('joy',Joy,JoystickCallback)
    rospy.init_node('servo_controller', anonymous=True)
    rate = rospy.Rate(100) #50 Hz is enough for this. 

     
    while not rospy.is_shutdown():
        servo1.set_target_speed(target_speed1)
        servo2.set_target_speed(target_speed2)
        servo1.update()
        servo2.update()
        servo3.set_target_speed(target_speed3)
        servo3.update()
        rate.sleep()                          
            

if __name__ == '__main__':
    servo_control()
