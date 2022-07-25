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
        self.trajectorytick = 0
        self.period = 1500 # gait cycle
        self.Vy = 0.0
        self.Vw = 0.0
        
        
        self.transformDuration = 1000
        self.initialPos = {} # initial pos of motors when entering the state
        
        self.mode = "stairs" # "stairs" ,"walk", "trot"
        
        if self.mode == "walk":
            self.targetPos = {
                "LF_Joint" :  -self.generate_position(self.period,1200,15000),
                "LB_Joint" :  -self.generate_position(self.period,1200,15000 + int(0.25*self.period)),
                "RF_Joint" :   self.generate_position(self.period,1200,15000 + int(0.75*self.period)),
                "RB_Joint" :   self.generate_position(self.period,1200,15000 + int(0.5*self.period))
            }
        elif self.mode == "stairs":
            self.targetPos = {
                "LF_Joint" :  self.generate_position_stairs(self.period,15000),
                "LB_Joint" :  self.generate_position_stairs(self.period,15000 + int(0.5*self.period)),
                "RF_Joint" :  -self.generate_position_stairs(self.period,15000),
                "RB_Joint" :  -self.generate_position_stairs(self.period,15000 + int(0.5*self.period))
            }
        elif self.mode == "trot":
            self.targetPos = {
                "LF_Joint" :  -self.generate_position(self.period,1200,15000),
                "LB_Joint" :  -self.generate_position(self.period,1200,15000 + int(0.5*self.period)),
                "RF_Joint" :   self.generate_position(self.period,1200,15000 + int(0.5*self.period)),
                "RB_Joint" :   self.generate_position(self.period,1200,15000)
            }
        
        self.changePos = {}
    
    def execute(self, userdata):
        
        self.tick = 0
        self.stateChangeFlag = False
        self.trajectorytick = 15000
        
        ## initialize to leg state canonical form
        for name in self.motorNameList:
            self.initialPos[name] = MotorManager.instance().getMotor(name).positionFdb # Get the initial position of the motor
            self.changePos[name] = (self.targetPos[name] - self.initialPos[name]) % (2.0*3.1415926)# calculate the change of angle of this motor ([-pi to pi])
            if(self.changePos[name]>3.1415926):
                self.changePos[name] = self.changePos[name] - 2.0*3.1415926
        
        r = rospy.Rate(1000)
        
        while(not self.stateChangeFlag):
            
            self.tick+=1
            
            if self.tick < self.transformDuration:
                for name in self.motorNameList:
                    # linear interp of motor position
                    # theta = theta_0 + (1-alpha)*theta_target
                    motor = MotorManager.instance().getMotor(name)
                    alpha = self.tick/float(self.transformDuration)
                    motor.positionSet = self.initialPos[name] + alpha*self.changePos[name]
            else:
                if self.joyData is not None:
                    self.Vw = 70.0 * self.joyData.axes[0]  
                    self.Vy = 1.5 * self.joyData.axes[1]
                    # self.Vy = 4.5 * abs(self.joyData.axes[1]) + 1.5*abs(self.joyData.axes[4])
                    self.Vy = min(4.5,self.Vy)
                    self.Vy = max(-4.5,self.Vy)
            
                self.trajectorytick += 1.0*self.Vy
                
                ## trot
        
                if self.mode == "walk":
                    ## walk
                    MotorManager.instance().getMotor("LF_Joint").positionSet = -self.generate_position(self.period,1200+self.Vw,self.trajectorytick)
                    MotorManager.instance().getMotor("LB_Joint").positionSet = -self.generate_position(self.period,1200+self.Vw,self.trajectorytick + int(0.25*self.period))
                    MotorManager.instance().getMotor("RF_Joint").positionSet =  self.generate_position(self.period,1200-self.Vw,self.trajectorytick + int(0.75*self.period))
                    MotorManager.instance().getMotor("RB_Joint").positionSet =  self.generate_position(self.period,1200-self.Vw,self.trajectorytick + int(0.5*self.period))            
                if self.mode == "stairs":
                    ## stairs
                    MotorManager.instance().getMotor("LF_Joint").positionSet = self.generate_position_stairs(self.period,self.trajectorytick)
                    MotorManager.instance().getMotor("LB_Joint").positionSet = self.generate_position_stairs(self.period,self.trajectorytick + int(0.5*self.period))
                    MotorManager.instance().getMotor("RF_Joint").positionSet = -self.generate_position_stairs(self.period,self.trajectorytick)
                    MotorManager.instance().getMotor("RB_Joint").positionSet = -self.generate_position_stairs(self.period,self.trajectorytick + int(0.5*self.period))            
                if self.mode == "trot":
                    MotorManager.instance().getMotor("LF_Joint").positionSet = -self.generate_position(self.period,1200+self.Vw,self.trajectorytick)
                    MotorManager.instance().getMotor("LB_Joint").positionSet = -self.generate_position(self.period,1200+self.Vw,self.trajectorytick + int(0.5*self.period))
                    MotorManager.instance().getMotor("RF_Joint").positionSet =  self.generate_position(self.period,1200-self.Vw,self.trajectorytick + int(0.5*self.period))
                    MotorManager.instance().getMotor("RB_Joint").positionSet =  self.generate_position(self.period,1200-self.Vw,self.trajectorytick)    
        
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