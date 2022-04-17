#!/usr/bin/env python
import rospy
from wheelleg_control.msg import JointData
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

def JoystickCallback(data):
    
    if(data.buttons[1]==1 and 
       (JoystickCallback.prevdata is None or JoystickCallback.prevdata.buttons[1]!=1)
       ): # flip legged state when B is pressed
        JoystickCallback.isLegged = not JoystickCallback.isLegged
        
    JoystickCallback.prevdata = data
    


if __name__ == '__main__':
    rospy.init_node('chassisController', anonymous=True)
    r = rospy.Rate(100)
    
    JoystickCallback.isLegged = False
    JoystickCallback.prevdata = None
    joySub = rospy.Subscriber('joy',Joy,JoystickCallback)
    jointPub = rospy.Publisher('/WheelLeg/command',JointData,queue_size=10)
    
    Vy = 0.0
    Vw = 0.0    
    
    
    while not rospy.is_shutdown():

        if(JoystickCallback.prevdata is not None):
            Vw = 25.0 * JoystickCallback.prevdata.axes[0]
            Vy = 25.0 * JoystickCallback.prevdata.axes[1]
                
        #Handle message
        msg = JointData()
        if(JoystickCallback.isLegged):
            msg.TransformLength = 10.0
        else:
            msg.TransformLength = 0.0
        

        for i in range(6):
            msg.JointMode.append(int(JoystickCallback.isLegged))
            if i < 3:
                msg.JointData.append(Vy-Vw)
            else:
                msg.JointData.append(Vy+Vw)
        
        print(Vw)
        
        #print(msg)
        
        jointPub.publish(msg)
        
        r.sleep()