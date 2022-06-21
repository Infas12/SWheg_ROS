#!/usr/bin/env python
import imp
import rospy
import smach
from States.WheelState import WheelState
from States.LegState import LegState
from States.TransitionState import TransitionState
from States.LeggedRelaxState import LeggedRelaxState

if __name__ == '__main__':
    rospy.init_node("StateMachine")
    sm = smach.StateMachine(outcomes=['exit'])
    with sm:
        smach.StateMachine.add('WHEEL',WheelState(),
                               transitions={'Transform':'TRANSITION'})
        smach.StateMachine.add('TRANSITION',TransitionState(),
                               transitions={'TransformCompleted':'LEGRELAX'})
        smach.StateMachine.add('LEGRELAX',LeggedRelaxState(),
                               transitions={'Walk':'STANDARD','Wheel':"WHEEL"})  
        smach.StateMachine.add('STANDARD',LegState(),
                               transitions={'Transform':'WHEEL','Stop':'LEGRELAX'})       
        
         
    outcome = sm.execute()
    

    
    
