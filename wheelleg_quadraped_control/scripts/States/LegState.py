#!/usr/bin/env python
from ast import Pass
import rospy
from States.Motor import MotorManager
from States.RobotState import RobotState, ContorlMode


class LegState(RobotState):
    
    def __init__(self):
        RobotState.__init__(self, outcomes=["Transform"])
        self.motorControlMode = ContorlMode.POS_MODE
        self.IsLeggedMode = True 
        self.tick = 0
        self.period = 1500 # gait cycle
        self.Vy = 0.0
        self.Vw = 0.0
    
    def execute(self, userdata):
        
        self.stateChangeFlag = False
        self.tick = 0
        r = rospy.Rate(1000)
        
        while(not self.stateChangeFlag):
            
            
            
            if self.joyData is not None:
                self.Vw = 90.0 * self.joyData.axes[3]  
                self.Vy = 2 * self.joyData.axes[4]
                self.Vy = min(2,self.Vy)
                self.Vy = max(-2,self.Vy)
            
            self.tick += 1.0*self.Vy
            
            # print(self.Vy)  
            
            ## trot
            MotorManager.instance().getMotor("LF_Joint").positionSet = -self.generate_position(self.period,1000+self.Vw,self.tick)
            MotorManager.instance().getMotor("LB_Joint").positionSet = -self.generate_position(self.period,1000+self.Vw,self.tick + int(0.5*self.period))
            MotorManager.instance().getMotor("RF_Joint").positionSet =  self.generate_position(self.period,1000-self.Vw,self.tick + int(0.5*self.period))
            MotorManager.instance().getMotor("RB_Joint").positionSet =  self.generate_position(self.period,1000-self.Vw,self.tick)            


            ## walk
            # MotorManager.instance().getMotor("LF_Joint").positionSet = -self.generate_position(self.period,1200+self.Vw,self.tick)
            # MotorManager.instance().getMotor("LB_Joint").positionSet = -self.generate_position(self.period,1200+self.Vw,self.tick + int(0.5*self.period))
            # MotorManager.instance().getMotor("RF_Joint").positionSet =  self.generate_position(self.period,1200-self.Vw,self.tick + int(0.25*self.period))
            # MotorManager.instance().getMotor("RB_Joint").positionSet =  self.generate_position(self.period,1200-self.Vw,self.tick + int(0.75*self.period))            


        
            self.sendData()
            
            r.sleep()
        
        return "Transform"
    
    
    def generate_position(self,period,time_stance,timestamp):
        
        PI = 3.1415926
        time_flight = period - time_stance

        turns = int(timestamp / period)
        x     = float(timestamp % period)
        
        theta_hit   = - (PI - 0.7)
        theta_leave = - 0.4
        
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
        
        if turns % 2 == 0: #even
            y = angle_in_one_period
        else: #odd
            y =  PI + angle_in_one_period
         
        return y