#!/usr/bin/env python
import rospy
import smach
from States.WheelState import WheelState
from States.LegState import LegState
from States.TransitionState import TransitionState
from States.TestState import TestState


if __name__ == '__main__':
    rospy.init_node("StateMachine")
    sm = smach.StateMachine(outcomes=['exit'])
    with sm:
        # smach.StateMachine.add('WHEEL',WheelState(),
        #                        transitions={'Transform':'TRANSITION'})
        # smach.StateMachine.add('TRANSITION',TransitionState(),
        #                        transitions={'TransformCompleted':'LEG'})
        # smach.StateMachine.add('LEG',LegState(),
        #                        transitions={'Transform':'WHEEL'})       
        smach.StateMachine.add('TEST',LegState(),
                               transitions={'Transform':'exit'})
         
    outcome = sm.execute()
    

    
    
