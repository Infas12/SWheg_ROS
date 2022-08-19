#!/usr/bin/env python
from ast import Pass

from std_msgs.msg import Float32
import rospy
from States.Motor import MotorManager
from RobotState import RobotState, ContorlMode


class StairState(RobotState):
    
    def __init__(self):
        RobotState.__init__(self, outcomes=["Wheel","Leg","Exit"])
        self.motorControlMode = ContorlMode.POS_MODE
        self.Mode = 1 
        self.tick = 0
        self.period = 1500 # gait cycle
        self.trajectoryTick = 10 * self.period
        self.Vy = 0.0
        self.Vw = 0.0
        
        self.transformDuration = 1000
        
        self.initialPos = {} # initial pos of motors when entering the state
        self.targetPos = {
            "LF_Joint" : self.generate_position_stairs(self.period,self.trajectoryTick),
            "LM_Joint" : self.generate_position_stairs(self.period,self.trajectoryTick + 0.33*self.period),
            "LB_Joint" : self.generate_position_stairs(self.period,self.trajectoryTick + 0.66*self.period),
            "RF_Joint" : -self.generate_position_stairs(self.period,self.trajectoryTick),
            "RM_Joint" : -self.generate_position_stairs(self.period,self.trajectoryTick + 0.33*self.period),
            "RB_Joint" : -self.generate_position_stairs(self.period,self.trajectoryTick + 0.66*self.period)
        }
        self.changePos = {}
        

    def execute(self, userdata):
        
        self.stateChangeFlag = False
        self.Initialized     = False
        self.trajectoryTick  = 10 * self.period
        self.tick            = 0
        r = rospy.Rate(1000)
        self.Apressed = False
        self.Bpressed = False
        self.Xpressed = False        
        self.Ypressed = False
        
        ## initialize to leg state canonical form
        for name in self.motorNameList:
            self.initialPos[name] = MotorManager.instance().getMotor(name).positionFdb # Get the initial position of the motor
            self.changePos[name] = (self.targetPos[name] - self.initialPos[name]) % (2.0*3.1415926)# calculate the change of angle of this motor ([-pi to pi])
            if(self.changePos[name]>3.1415926):
                self.changePos[name] = self.changePos[name] - 2.0*3.1415926
                
            
        while(not self.stateChangeFlag):
            
            self.tick += 1
            
            if self.tick < self.transformDuration:
                for name in self.motorNameList:
                    # linear interp of motor position
                    # theta = theta_0 + (1-alpha)*theta_target
                    motor = MotorManager.instance().getMotor(name)
                    alpha = self.tick/float(self.transformDuration)
                    motor.positionSet = self.initialPos[name] + alpha*self.changePos[name]
            
            else:
                # handle joystick command
                if self.joyData is not None:
                    self.Vw = 700.0 * self.joyData.axes[0]  
                    self.Vy = 1.0 * self.joyData.axes[1]
                    self.Vy = min(3,self.Vy)
                    self.Vy = max(-3,self.Vy) 
                    
                # the trajectory is clock-driven.
                self.trajectoryTick += self.Vy * 1.0
                
                # set position
                MotorManager.instance().getMotor("LF_Joint").positionSet = self.generate_position_stairs(self.period,self.trajectoryTick)
                MotorManager.instance().getMotor("LM_Joint").positionSet = self.generate_position_stairs(self.period,self.trajectoryTick + 0.33*self.period)
                MotorManager.instance().getMotor("LB_Joint").positionSet = self.generate_position_stairs(self.period,self.trajectoryTick + 0.66*self.period)
                MotorManager.instance().getMotor("RF_Joint").positionSet = -self.generate_position_stairs(self.period,self.trajectoryTick)
                MotorManager.instance().getMotor("RM_Joint").positionSet = -self.generate_position_stairs(self.period,self.trajectoryTick + 0.33*self.period)
                MotorManager.instance().getMotor("RB_Joint").positionSet = -self.generate_position_stairs(self.period,self.trajectoryTick + 0.66*self.period)            
            
            if(self.Apressed):
                return "Wheel"        
        
            if(self.Bpressed):
                return "Leg"     
            
            if(self.Ypressed):
                return "Exit"         
            
            self.sendData()            
            r.sleep()
        

    
    
    def generate_position_stairs(self,period,timestamp):

        PI = 3.1415926

        turns = int(timestamp / period)
        x     = float(timestamp % period)
        k = float(PI/period)
        angle_in_one_period = float(k * x)
        y = 0
        
        if turns % 2 == 0: #even
            y = angle_in_one_period
        else: #odd
            y =  PI + angle_in_one_period
         
        return y