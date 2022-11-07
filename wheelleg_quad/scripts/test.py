#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from servo import servo
from scservo_sdk import *
import numpy as np
import sys
target_position_1 = {"Open":3250,"Close":150}
target_position_2 = {"Open":3500,"Close":500}
state = "Close"



def JoystickCallback(data):
    global state
    if(data.buttons[1]==1):
        if state == "Close":
            state = "Open"
        else:
            state = "Close"
        print(target_position_2[state])

def servo_control():
    global state
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
    rate = rospy.Rate(1000) #50 Hz is enough for this. 

    while not rospy.is_shutdown():
        
        servo1.set_target_position(target_position_1[state])
        servo2.set_target_position(target_position_2[state])

        # #very rough constant-speed control
        if(abs(servo1.present_position-servo1.target_position)<50):
            servo1.set_target_speed(0)
        elif(servo1.present_position>servo1.target_position):
            servo1.set_target_speed(32968)
        elif(servo1.present_position<servo1.target_position):
            servo1.set_target_speed(150)
           

        if(abs(servo2.present_position-servo2.target_position)<50):
            servo2.set_target_speed(0)
        elif(servo2.present_position>servo2.target_position):
            servo2.set_target_speed(32968)
        elif(servo2.present_position<servo2.target_position):
            servo2.set_target_speed(150)
        
        servo1.update()
        servo2.update()
    rate.sleep()

    # while 1:
    #     print("Press any key to continue! (or press ESC to quit!)")
    #     if getch() == chr(0x1b):
    #         break

    #     servo1.set_target_position(goal_position[index])
    #     servo2.set_target_position(goal_position[index])

    #     while 1:
            
    #         servo1.update()
    #         servo2.update()

    #         if servo1.reached_target_pos() and servo2.reached_target_pos():
    #             break

    #     # Change goal position
    #     if index == 0:
    #         index = 1
    #     else:
    #         index = 0    

if __name__ == '__main__':
    servo_control()