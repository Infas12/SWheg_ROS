#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import numpy as np
import sys
import RPi.GPIO as GPIO

state = "stop"


def JoystickCallback(data):
    global state
    if(data.buttons[4]==1): #LB
        if(data.buttons[0]==1): #A
            state = "wheel"
        elif data.buttons[1]==1: #B
            state = "leg"
        else:
            state = "stop"
    else:
        state = "stop"
    # rospy.logwarn(" %d %d %d" ,data.axes[7], data.axes[7] ,data.buttons[6])

def linear_control():
    global state

    sub = rospy.Subscriber('joy',Joy,JoystickCallback)
    rospy.init_node('linear_controller', anonymous=True)
    rate = rospy.Rate(50) 
    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    # 为输出的引脚设置默认值
    chan_list = [35,36,37,38]
    GPIO.setup(chan_list, GPIO.OUT, initial=0)

    while not rospy.is_shutdown():
        if state == "wheel":
            GPIO.output(chan_list, (1,0,0,1))   
        elif state == "leg" :
            GPIO.output(chan_list, (0,1,1,0))
        elif state == "stop":
            GPIO.output(chan_list, (1,1,0,0))
        rate.sleep()

    GPIO.cleanup()



if __name__ == '__main__':
    linear_control()