#!/usr/bin/env python
import imp
import rospy
import smach
from WheelState import WheelState
from LegState import LegState
from TransitionState import TransitionState

if __name__ == '__main__':
    rospy.init_node("StateMachine")
    sm = smach.StateMachine(outcomes=['exit'])
    with sm:
        smach.StateMachine.add('WHEEL',WheelState(),
                               transitions={'Transform':'TRANSITION'})
        smach.StateMachine.add('TRANSITION',TransitionState(),
                               transitions={'TransformCompleted':'LEG'})
        smach.StateMachine.add('LEG',LegState(),
                               transitions={'Transform':'WHEEL'})       
        
         
    outcome = sm.execute()
    

    
    
