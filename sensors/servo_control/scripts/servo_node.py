#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from servo import servo
from scservo_sdk import *
import numpy as np
import sys
target_position_1 = {"Open":3250,"Close":150}
target_position_2 = {"Open":3500,"Close":500}
target_speed = 0 # 32768


def JoystickCallback(data):
    global target_speed

    if(data.buttons[4]==1): #LB
        if(data.buttons[2]==1):#x 顺时针
            target_speed = 500
        elif(data.buttons[3]==1):#y 逆时针
            target_speed = 32768 + 500
        else:
            target_speed = 0
    else:
        target_speed = 0

        

def servo_control():
    global target_speed

    # Serial Setup
    BAUDRATE                    = 500000           
    DEVICENAME                  = '/dev/ttyUSB0'   
    protocol_end                = 0           # SCServo bit end(STS/SMS=0, SCS=1)

    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(protocol_end)
    portHandler.openPort()
    portHandler.setBaudRate(BAUDRATE)
    servo1 = servo(1,packetHandler,portHandler)
    servo2 = servo(2,packetHandler,portHandler)

    sub = rospy.Subscriber('joy',Joy,JoystickCallback)
    rospy.init_node('servo_controller', anonymous=True)
    rate = rospy.Rate(100) #50 Hz is enough for this. 

     
    while not rospy.is_shutdown():
        servo1.set_target_speed(target_speed)
        servo2.set_target_speed(target_speed)
        servo1.update()
        servo2.update()
        rate.sleep()                          
            

if __name__ == '__main__':
    servo_control()
