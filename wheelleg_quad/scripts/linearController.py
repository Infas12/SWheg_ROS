#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import numpy as np
import sys
import RPi.GPIO as GPIO

state = "nothing"


def JoystickCallback(data):
    global state

    if(data.axes[7]==1): #Cross key up/down
        state = "forward"
    elif data.axes[7]==-1: #Cross key up/down
        state = "backward"
    elif data.buttons[6]==1: #back
        state = "stop"
    else:
        state = "nothing"
    # rospy.logwarn(" %d %d %d" ,data.axes[7], data.axes[7] ,data.buttons[6])

def linear_control():
    global state

    sub = rospy.Subscriber('joy',Joy,JoystickCallback)
    rospy.init_node('linear_controller', anonymous=True)
    rate = rospy.Rate(50) 

    chan_list = [35,36,37,38]
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    # 为输出的引脚设置默认值
    GPIO.setup(35, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(36, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(37, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(38, GPIO.OUT, initial=GPIO.LOW)

    while not rospy.is_shutdown():
        if state == "forward":
            GPIO.output(chan_list, (1,0,0,1))   
            rospy.logwarn(" forward" )
        elif state == "backward" :
            GPIO.output(chan_list, (0,1,1,0))
            rospy.logwarn(" backward" )
        elif state == "stop":
            GPIO.output(chan_list, (1,1,0,0))
            rospy.logwarn(" stop" )
        elif state == "nothing":
            GPIO.output(chan_list, (0,0,0,0))
            rospy.logwarn(" nothing" )
        rate.sleep()

    GPIO.cleanup()



if __name__ == '__main__':
    linear_control()