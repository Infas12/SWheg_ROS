from scservo_sdk import *

# Most of the functions has NOT been implemented yet.


# Address table
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_SCS_GOAL_ACC = 41
ADDR_SCS_GOAL_POSITION = 42
ADDR_SCS_GOAL_SPEED = 46
ADDR_SCS_PRESENT_POSITION = 56
ADDR_SCS_PRESENT_VOLTAGE = 62
ADDR_SCS_PRESENT_CURRENT = 69

# Servo attribute.
SCS_MINIMUM_POSITION_VALUE = 0           # SCServo will rotate between this value
# and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
SCS_MAXIMUM_POSITION_VALUE = 4095
SCS_MOVING_STATUS_THRESHOLD = 20          # SCServo moving status threshold

SCS_MINIMUM_SPEED_VALUE = 36968
SCS_MAXIMUM_SPEED_VALUE = 4000

protocol_end = 0           # SCServo bit end(STS/SMS=0, SCS=1)


class servo:

    def __init__(self, ID, packet, port):
        self.ID = ID
        self.portHandler = port
        self.packetHandler = packet

        self.target_position = 0   #int,range from 0 to SCS_MAXIMUM_POSITION_VALUE
        self.target_speed = 0   #int,range from 0 to 300

        self.present_position = 0
        self.present_speed = 0
        self.click_num = 0

        #TODO: Replace this with Enum
        self.control_mode = 1 #0-position mode 1-speed close-loop mode 

    def set_target_position(self, pos):
        pos = min(pos, SCS_MAXIMUM_POSITION_VALUE)
        pos = max(SCS_MINIMUM_POSITION_VALUE, pos)
        self.target_position = pos
    
    def set_target_speed(self,spd):

        self.target_speed = spd
        
    def set_click_num(self,click_num):
        self.click_num = click_num

    def get_pos(self):
        present_position_speed, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, self.ID, ADDR_SCS_PRESENT_POSITION)
        self.present_position = SCS_LOWORD(present_position_speed) 
        return self.present_position

    def get_voltage(self):
        temp, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.ID, ADDR_SCS_PRESENT_VOLTAGE)
        self.present_voltage = SCS_LOWORD(temp) 
        return self.present_voltage

    def get_current(self):
        temp, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.ID, ADDR_SCS_PRESENT_CURRENT)
        self.present_current = SCS_LOWORD(temp) 
        return self.present_current

    def set_target_position1(self,state1):
        
        if self.click_num ==1 :
            change_position =3000
            if state1 =="Close":
                change_position =-3000
                self.target_speed = 200
                
            elif state1 == "Open":
                self.target_speed = 32968
                
                
            present_position_speed, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, self.ID, ADDR_SCS_PRESENT_POSITION)
            self.present_position = SCS_LOWORD(present_position_speed) 
            
            if self.present_position +change_position >4095:
                self.target_position = self.present_position -1095
            elif self.present_position +change_position < 0 :
                self.target_position = self.present_position + 1095
            else:
                self.target_position = self.present_position + change_position   
            
        elif self.click_num ==0:
            self.target_position = self.target_position
            



        
        
        
    #TODO: enable mode change. Haven't read through the datasheet yet.
    def set_control_mode(self,mode):
        if mode == 0:
            pass
        elif mode == 1:
            pass
        self.mode = mode

    def update(self):
        # update feedback
        present_position_speed, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, self.ID, ADDR_SCS_PRESENT_POSITION) #request feedback
        self.present_position = SCS_LOWORD(present_position_speed)
        self.present_speed = SCS_HIWORD(present_position_speed)
        
        if self.control_mode == 0:
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(
                self.portHandler, self.ID, ADDR_SCS_GOAL_POSITION, self.target_position)
            # print("ID",self.ID,"pre_pos:",self.present_position,"tar_pos",self.target_position)        

        elif self.control_mode == 1:
            # scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(
            #     self.portHandler,self.ID,ADDR_SCS_GOAL_ACC,1)            
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(
                self.portHandler, self.ID, ADDR_SCS_GOAL_SPEED, self.target_speed)
            # if(abs(self.target_speed)>100):
            #     scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(
            #         self.portHandler, self.ID, ADDR_SCS_GOAL_ACC, 1)
            # else: #brake
                
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(
                self.portHandler, self.ID, ADDR_SCS_GOAL_ACC, 30)
            
            # print("ID:",self.ID,", pre_pos:",self.present_position,", tar_pos",self.target_position,", pre_spd",self.present_speed)                

    def reached_target_pos(self):
        return abs(self.target_position - self.present_position) < SCS_MOVING_STATUS_THRESHOLD
