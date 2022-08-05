#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import serial
import crcmod
import time
import struct
import sys

# CRC16校验，返回整型数
def crc16(veritydata):
    if not veritydata:
        return
    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    return crc16(veritydata)

# 校验数据帧的CRC码是否正确
def checkcrc(data):
    if not data:
        return False
    if len(data) <= 2:
        return False
    nocrcdata = data[:-2]
    oldcrc16 = data[-2:]
    oldcrclist = list(oldcrc16)
    crcres = crc16(nocrcdata)
    crc16byts = crcres.to_bytes(2, byteorder="little", signed=False)
    # print("CRC16:", crc16byts.hex())
    crclist = list(crc16byts)
    if oldcrclist[0] != crclist[0] or oldcrclist[1] != crclist[1]:
        return False
    return True

# Modbus-RTU协议的03或04读取保存或输入寄存器功能主-》从命令帧
def mmodbus03or04(add, startregadd, regnum, funcode=3):
    # if add < 0 or add > 0xFF or startregadd < 0 or startregadd > 0xFFFF or regnum < 1 or regnum > 0x7D:
    #     print("Error: parameter error")
    #     return
    # if funcode != 3 and funcode != 4:
    #     print("Error: parameter error")
    #     return
    sendbytes = add.to_bytes(1, byteorder="big", signed=False)
    sendbytes = sendbytes + funcode.to_bytes(1, byteorder="big", signed=False) + startregadd.to_bytes(2, byteorder="big", signed=False) + \
                regnum.to_bytes(2, byteorder="big", signed=False)
    crcres = crc16(sendbytes)
    crc16bytes = crcres.to_bytes(2, byteorder="little", signed=False)
    sendbytes = sendbytes + crc16bytes
    return sendbytes

# Modbus-RTU协议的03或04读取保持或输入寄存器功能从-》主的数据帧解析
# valueformat=0：返回的数据是32位有符号整型（功率传感器）
def smodbus03or04(recvdata, valueformat=0, intsigned=False):
    if not recvdata:
        print("Error: data error")
        return
    if not checkcrc(recvdata):
        print("Error: crc error")
        return
    datalist = list(recvdata)
    if datalist[1] != 0x3 and datalist[1] != 0x4:
        print("Error: recv data funcode error")
        return
    bytenums = datalist[2]
    if bytenums % 2 != 0:
        print("Error: recv data reg data error")
        return
    retdata = []
    if valueformat == 0:
        signedintnums = bytenums / 4
        # print("32signed int nums: ", str(signedintnums))
        for i in range(int(signedintnums)):
            btemp = recvdata[3+i*4:7+i*4]
            intvalue = int.from_bytes(btemp, byteorder="big", signed=intsigned)
            retdata.append(intvalue)
            # print(f"Data{i+1}: {intvalue}")
    return retdata


class PowerSensor:
    def __init__(self):
        self.joySub   = rospy.Subscriber('joy',Joy,self.JoystickCallback)
        self.joyData  = None

        self.dataFinish = True
        self.powerFile = 0

        self.slaveadd = 1
        self.startreg = 0x0BB8 # voltage measurement register
        self.regnums = 8
        self.send_data = mmodbus03or04(self.slaveadd, self.startreg, self.regnums)
        # print("send data : ", send_data.hex())
        self.com = serial.Serial("/dev/ttyUSB1", 115200, timeout=0.8)
        print("com open")

    def JoystickCallback(self,data): # power button
        if(data.buttons[8]==1 and 
        (self.joyData is None or self.joyData.buttons[8]!=1) # compare current joystick data with previous data
        ): 
            if(self.dataFinish):
                rospy.logwarn("write start")
                recordname = time.strftime("%Y%m%d%H%M%S", time.localtime()) + '.txt'
                self.powerFile = open(recordname,"w")
                self.dataFinish = False
            else:
                self.dataFinish = True
                rospy.logwarn("write finished")
                self.powerFile.close()
        if(data.buttons[6]==1 and
        (self.joyData is None or self.joyData.buttons[6]!=1) # compare current joystick data with previous data
        ): 
            self.com.close()
            print("com close")
        self.joyData = data # collect joystick data



    def getPowerNode(self):
        rospy.init_node('getPowerNode')
        r = rospy.Rate(20) #upper limit: 20Hz

        while not rospy.is_shutdown():
            if(not self.dataFinish):
                # starttime = time.time()
                self.com.write(self.send_data)
                recv_data = self.com.read(self.regnums*2+5)
                # endtime = time.time()
                # if len(recv_data) > 0:
                #     print("recv: ", recv_data.hex())
                # print(f"used time: {endtime-starttime:.3f}")

                sensorValueList = smodbus03or04(recv_data)
                #sensorValueList: [24529,    194,     4752,    26887]
                #real:             24.529V   0.194A   4.752W   2.68Wh

                strList = [str(i) for i in sensorValueList]
                myString = " ".join(strList)

                nowTime = rospy.get_time()
                timeStr = str(("%.7f"% nowTime)) + "  " # str(nowTime.secs) + "." + str(nowTime.nsecs) + "  "
                myString = timeStr + myString + "\n"
                # print(myString) 
                self.powerFile.write(myString)
            r.sleep()

        rospy.spin()


if __name__ == '__main__':
    powerNode = PowerSensor()
    powerNode.getPowerNode()

# 其他设置
# 波特率寄存器
#   9600: 01 10 0C 1C 00 01 02 00 02 E9 CD      current: 0.036s
#   115200: 01 10 0C 1C 00 01 02 00 06 E8 0E    current: 0.007s

# 设置频率20Hz：01 10 0C 81 00 01 02 00 14 74 4E
# 电量双向累计：01 10 0C 82 00 01 02 00 03 34 73
# 从电量单位都所有： 01 03 0C 80 00 04 46 B1
#               返回值：01 03 08 00 00 00 14 00 03 00 00 55 d4
#                      电量单位wh，系统采样频率20，电量双向累计，电压档位自动挡
