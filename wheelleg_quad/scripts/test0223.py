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
state1 = "Close"
state  = 0
servoNum = 1
click_num = 0

axes6R = False
ofile = 0

def JoystickCallback(data):
    global state1
    global state
    global servoNum
    global target_speed
    global click_num
    global axes6R
    global ofile

    if(data.buttons[0]==1):
        state = 0
        target_speed = 0
    elif data.buttons[1]==1:
        state = 1
        if state1 == "Close":
            state1 = "Open"
            click_num = 1
        else:
            state1 = "Close"
            click_num =1
    elif(data.buttons[2]==1):#x 顺时针
        state = 2
        target_speed = 200
    elif(data.buttons[3]==1):#y
        state = 3 
        target_speed = 32968
    elif(data.buttons[4]==1):
        state = 4
        servoNum = 1
    elif(data.buttons[5]==1):
        state = 5
        servoNum = 2

    if(data.axes[6]==-1):#cros key L/R
        if(axes6R == False):
            print("write")
            ofile = open("servo_vol_cur.txt", "w") 
            axes6R = True 
        else:
            print("write done")
            axes6R = False 
            ofile.close()
            
        

def servo_control():
    global state1
    # Serial Setup
    global state
    global servoNum
    global target_speed
    global click_num
    global axes6R
    global ofile

    target_speed = 0 # 32768

    state  = 1
    servoNum = 2

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
        if state == 1:
            servo1.set_click_num(click_num)
            servo2.set_click_num(click_num)
            click_num=0
            servo1.set_target_position1(state1)
            servo2.set_target_position1(state1)
            # #very rough constant-speed control
            if(abs(servo1.present_position-servo1.target_position)<50):
                servo1.set_target_speed(0)
            # elif(servo1.present_position>servo1.target_position):
            #     servo1.set_target_speed(32968)
            # elif(servo1.present_position<servo1.target_position):
            #     servo1.set_target_speed(150)
            if(abs(servo2.present_position-servo2.target_position)<50):
                servo2.set_target_speed(0)
            # elif(servo2.present_position>servo2.target_position):
                
            #     servo2.set_target_speed(32968)
            # elif(servo2.present_position<servo2.target_position):
            #     servo2.set_target_speed(150)
            
            servo1.update()
            servo2.update()
            rate.sleep()
        elif state == 0 :
            servo1.set_target_speed(0)
            servo2.set_target_speed(0)
            servo1.update()
            servo2.update()
            rate.sleep()
        elif state == 2 :
            if servoNum == 1:
                servo1.set_target_speed(target_speed)
                servo1.update()
                rate.sleep()
            elif servoNum == 2:
                servo2.set_target_speed(target_speed)
                servo2.update()
                rate.sleep()
        elif state == 3 :
            if servoNum == 1:
                servo1.set_target_speed(target_speed)
                servo1.update()
                rate.sleep()
            elif servoNum == 2:
                servo2.set_target_speed(target_speed)
                servo2.update()
                rate.sleep() 
        elif state == 4 or state == 5:
            
            servo1.set_target_speed(0)
            servo2.set_target_speed(0)

            servo1.update()
            servo2.update()
            rate.sleep()                          
            
        # output attributes to txt
        if(axes6R == True):##FIXME: little bug
            print(servo1.get_voltage(), servo1.get_current(), servo2.get_voltage(), servo2.get_current() ,file=ofile)

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