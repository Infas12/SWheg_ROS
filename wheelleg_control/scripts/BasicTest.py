#!/usr/bin/env python
import rospy
from wheelleg_control.msg import JointState
from sensor_msgs.msg import Joy


def JoystickCallback(data):
    
    if(data.buttons[1]==1 and 
       (JoystickCallback.prevdata is None or JoystickCallback.prevdata.buttons[1]!=1)
       ):
        JoystickCallback.isLegged = not JoystickCallback.isLegged
        
    JoystickCallback.prevdata = data


if __name__ == '__main__':
    rospy.init_node('chassisController', anonymous=True)
    r = rospy.Rate(100)
    
    JoystickCallback.isLegged = False
    JoystickCallback.prevdata = None
    joySub = rospy.Subscriber('joy',Joy,JoystickCallback)
    jointPub = rospy.Publisher('/WheelLeg/command',JointState,queue_size=10)
    
    Vy = 0.0
    Vw = 0.0    
    
    
    
    while not rospy.is_shutdown():

        if(JoystickCallback.prevdata is not None):
            Vw = 25.0 * JoystickCallback.prevdata.axes[0]
            Vy = -25.0 * JoystickCallback.prevdata.axes[1]
        
        msg = JointState()
        for i in range(6):
            msg.JointMode.append(0)
            msg.JointData.append(Vy)
        
        print(msg)
        
        jointPub.publish(msg)
        
        r.sleep()