#!/usr/bin/env python
import imp
import rospy
import smach
from States.WheelState import WheelState
from States.LegState import LegState
from States.StairState import StairState
# from States.StairState1 import StairState1

if __name__ == '__main__':
    rospy.init_node("StateMachine")
    sm = smach.StateMachine(outcomes=['exit'])
    with sm:
        smach.StateMachine.add('WHEEL',WheelState(),
                               transitions={'Leg':'LEG','Stairs':'STAIR','Exit':'exit'})
        smach.StateMachine.add('LEG',LegState(),
                               transitions={'Wheel':'WHEEL','Stairs':'STAIR','Exit':'exit'})       
        # smach.StateMachine.add('LEG',StairState1(),
        #                        transitions={'Wheel':'WHEEL','Stairs':'STAIR','Exit':'exit'})                                     
        smach.StateMachine.add('STAIR',StairState(),
                               transitions={'Wheel':'WHEEL','Leg':'LEG','Exit':'exit'})
         
    outcome = sm.execute()
    

    
    
