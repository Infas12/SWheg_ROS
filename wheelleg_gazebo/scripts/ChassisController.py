#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64


if __name__ == '__main__':    
    
    name_list = ["LFRA","LMRA","LBRA","RFRA","RMRA","RBRA",
                 "LFRB","LMRB","LBRB","RFRB","RMRB","RBRB"]
    rospy.init_node('chassisController', anonymous=True)
    transformer_joint_publisher_list = []
    for name in name_list:
        transformer_joint_publisher_list.append(
            rospy.Publisher('/WheelLeg/' + name +'_position_controller/command',Float64,queue_size=10)
        )
    
    r = rospy.Rate(10)
    counter = 0
    joint_angle = 0.0
    legged = False
    while not rospy.is_shutdown():
        
        if legged:
            joint_angle = -0.5
        else:
            joint_angle = 0.0
        
        for pub in transformer_joint_publisher_list:
            pub.publish(joint_angle)
        
        if counter % 20 == 0:
            legged = not legged
        
        print(counter)
        
        counter += 1
        r.sleep()
        