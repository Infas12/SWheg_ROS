#!/usr/bin/env python
import imp
import rospy
import smach
from WheelState import WheelState

if __name__ == '__main__':
    rospy.init_node("StateMachine")
    sm = smach.StateMachine(outcomes=['exit'])
    with sm:
        smach.StateMachine.add('WHEEL',WheelState(),
                               transitions={'Transform':'exit'})
    outcome = sm.execute()
    

    
    
