import rospy
from wheelleg_control.msg import JointData
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from Motor import MotorManager,Motor
WHEEL_MODE = 0
LEG_MODE = 1

class ChassisController:

    def __init__(self):
        self.mode = WHEEL_MODE
        self.motorNameList = ["LF_Joint","LM_Joint","LB_Joint","RB_Joint","RM_Joint","RF_Joint"]
        self.motorList = []
        self.vy = 0.0
        self.vw = 0.0
        for name in self.motorNameList:
            self.motorList.append(Motor(name))            
        self.jointPub = rospy.Publisher('/WheelLeg/command',JointData,queue_size=10)
        self.joySub = rospy.Subscriber('joy',Joy,self.JoystickCallback)
        self.prevdata = None
    
    def update(self):
        
        if self.prevdata is not None:
            self.vw = 25.0 * self.prevdata.axes[0]
            self.vy = 25.0 * self.prevdata.axes[1]

        self.motorList[0].speedSet = (self.vy-self.vw)*1.0
        self.motorList[1].speedSet = (self.vy-self.vw)*1.0
        self.motorList[2].speedSet = (self.vy-self.vw)*1.0
        self.motorList[3].speedSet = (self.vy+self.vw)*-1.0
        self.motorList[4].speedSet = (self.vy+self.vw)*-1.0
        self.motorList[5].speedSet = (self.vy+self.vw)*-1.0        
        
        self.sendData()


    def sendData(self):
        msg = JointData()
        if(self.mode == LEG_MODE):
            msg.TransformLength = 10.0
        else:
            msg.TransformLength = 0.0
            
        for motor in self.motorList:
            msg.JointMode.append(int(self.mode))
            if self.mode == WHEEL_MODE:
                msg.JointData.append(motor.speedSet)
            else:
                msg.JointData.append(motor.positionSet)
        self.jointPub.publish(msg)
        
        
    def JoystickCallback(self,data):    
        if(data.buttons[1]==1 and 
        (self.prevdata is None or self.prevdata.buttons[1]!=1)
        ): # flip legged state when B is pressed
            self.mode = 1 - self.mode
        self.prevdata = data