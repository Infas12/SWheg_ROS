#!/usr/bin/env python
from ast import Pass

from std_msgs.msg import Float32
import rospy
from States.Motor import MotorManager
from RobotState import RobotState, ContorlMode


class LegState(RobotState):
    
    def __init__(self):
        RobotState.__init__(self, outcomes=["Wheel","Stairs","Exit"])
        self.motorControlMode = ContorlMode.POS_MODE
        self.IsLeggedMode = True 
        
        self.tick = 0
        self.period = 1500 # gait cycle
        self.trajectoryTick = 10*self.period
        self.Vy = 0.0
        self.Vw = 0.0
        
        self.transformDuration = 1000
        
        self.initialPos = {} # initial pos of motors when entering the state
        self.targetPos = {
            "LF_Joint" : -self.generate_position(self.period,1000,self.trajectoryTick + (0.5*self.period)),
            "LM_Joint" : -self.generate_position(self.period,1000,self.trajectoryTick),
            "LB_Joint" : -self.generate_position(self.period,1000,self.trajectoryTick + (0.5*self.period)),
            "RF_Joint" :  self.generate_position(self.period,1000,self.trajectoryTick),
            "RM_Joint" :  self.generate_position(self.period,1000,self.trajectoryTick + (0.5*self.period)),
            "RB_Joint" :  self.generate_position(self.period,1000,self.trajectoryTick)
        }
        self.changePos = {}
        

    def execute(self, userdata):
        
        self.stateChangeFlag = False
        self.Initialized     = False
        self.trajectoryTick  = 10*self.period
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
                print(self.Vy)
                
                # set position
                # MotorManager.instance().getMotor("LF_Joint").positionSet = -self.generate_position(self.period,1000+self.Vw,self.trajectoryTick + int(0.5*self.period))
                # MotorManager.instance().getMotor("LM_Joint").positionSet = -self.generate_position(self.period,1000+self.Vw,self.trajectoryTick)
                # MotorManager.instance().getMotor("LB_Joint").positionSet = -self.generate_position(self.period,1000+self.Vw,self.trajectoryTick + int(0.5*self.period))
                # MotorManager.instance().getMotor("RF_Joint").positionSet =  self.generate_position(self.period,1000-self.Vw,self.trajectoryTick)
                # MotorManager.instance().getMotor("RM_Joint").positionSet =  self.generate_position(self.period,1000-self.Vw,self.trajectoryTick + int(0.5*self.period))
                # MotorManager.instance().getMotor("RB_Joint").positionSet =  self.generate_position(self.period,1000-self.Vw,self.trajectoryTick)            

                MotorManager.instance().getMotor("LF_Joint").positionSet = -self.generate_position(self.period,1000+self.Vw,self.trajectoryTick)
                MotorManager.instance().getMotor("LM_Joint").positionSet = -self.generate_position(self.period,1000+self.Vw,self.trajectoryTick + int(0.33*self.period))
                MotorManager.instance().getMotor("LB_Joint").positionSet = -self.generate_position(self.period,1000+self.Vw,self.trajectoryTick + int(0.66*self.period))
                MotorManager.instance().getMotor("RB_Joint").positionSet =  self.generate_position(self.period,1000-self.Vw,self.trajectoryTick + int(0.165*self.period))
                MotorManager.instance().getMotor("RM_Joint").positionSet =  self.generate_position(self.period,1000-self.Vw,self.trajectoryTick + int(0.495*self.period))
                MotorManager.instance().getMotor("RF_Joint").positionSet =  self.generate_position(self.period,1000-self.Vw,self.trajectoryTick + int(0.825*self.period))  
                
            if(self.Apressed):
                return "Wheel"        
        
            if(self.Xpressed):
                return "Stairs"            
            
            if(self.Ypressed):
                return "Exit"  
            
            self.sendData()            
            r.sleep()
        

    
    
    def generate_position(self,period,time_stance,timestamp):
        
        PI = 3.1415926
        theta_hit   = - (PI - 0.7)
        theta_leave = - 0.4        

        time_flight = period - time_stance

        turns = int(timestamp / period)
        x     = float(timestamp % period)
        
        k_flight = float(theta_hit - theta_leave) / float(time_flight)
        k_stance = float(- PI - theta_hit + theta_leave) / float(time_stance)
        
        t_leave  = theta_leave / float(k_stance)
        t_hit    = t_leave + time_flight
        
        angle_in_one_period = 0
        
        if x <= t_leave:
            angle_in_one_period = float(k_stance * x)
        elif x > t_leave and x <= t_hit:
            angle_in_one_period = theta_leave + (x - t_leave) * k_flight
        else :
            angle_in_one_period = theta_hit + (x - t_hit) * k_stance
        
        y = 0
        
        if turns >= 0:
            if turns % 2 == 0: #even
                y = angle_in_one_period
            else: #odd
                y =  PI + angle_in_one_period
        else:
            if turns % 2 == 0: #even
                y = angle_in_one_period
            else: #odd
                y = PI + angle_in_one_period            
        
        
        return y