#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import rospy
import time
import math
import sys
import platform
import threading
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from std_msgs.msg import String
# from tf.transformations import quaternion_from_euler


# 查找 ttyUSB* 设备
def find_ttyUSB():
    print('imu deflaut serial: /dev/ttyUSB0, please modify')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('the {} device lenth {} are {}'.format('USB', len(posts), posts))

# 校验
def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


# 16 进制转 ieee 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag, readreg, calibuff, flag, mag_offset, mag_range, version
    angle_flag=False
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
        return
    else:
        data_buff = list(buff.values())  # 获取字典所有 value
        if buff[1] == 0x51 :
            if checkSum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
            else:
                print('0x51 get failed')

        elif buff[1] == 0x52:
            if checkSum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]

            else:
                print('0x52 get failed')

        elif buff[1] == 0x53:
            if checkSum(data_buff[0:10], data_buff[10]):
                temp = hex_to_short(data_buff[2:10])
                angle_degree = [temp[i] / 32768.0 * 180 for i in range(0, 3)]
                version = temp[3]
                angle_flag = True
            else:
                print('0x53 get failed')
        elif buff[1] == 0x54:
            if checkSum(data_buff[0:10], data_buff[10]):
                magnetometer = hex_to_short(data_buff[2:10])
                if flag:
	                calibuff.append(magnetometer[0:2])
            else:
                print('0x54 get failed')
        elif buff[1] == 0x5f:
            if checkSum(data_buff[0:10], data_buff[10]):
                readval = hex_to_short(data_buff[2:10])
                if readreg == 0x0b:
	                mag_offset = readval
                else:
	                mag_range = readval

                print(readval)
            else:
                print('0x5f get failed')

        else:
            #print("该数据处理类没有提供该 " + str(buff[1]) + " 的解析")
            #print("或数据错误")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if angle_flag:
            imu_msg.linear_acceleration.x = acceleration[0]
            imu_msg.linear_acceleration.y = acceleration[1]
            imu_msg.linear_acceleration.z = acceleration[2]

            imu_pub.publish(imu_msg)


version = 0
readreg = 0
key = 0
flag = 0
iapflag = 0
recordflag = 0
buff = {}
calibuff = list()
recordbuff = list()
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
mag_offset = [0, 0, 0]
mag_range = [0, 0, 0]
wt_imu = serial.Serial()
baudlist = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]




def thread_job():
    print("thread run")
    rospy.spin()



if __name__ == "__main__":
    # global recordflag, recordbuff, wt_imu

    python_version = platform.python_version()[0]

    find_ttyUSB()
    rospy.init_node("imu")
    r = rospy.Rate(200)
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baud", 9600)
    # baudrate = 115200
    print("IMU Type: Normal Port:%s baud:%d" %(port,baudrate))
    imu_msg = Imu()
    mag_msg = MagneticField()
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    try:
        wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=10)
        if wt_imu.isOpen():
            rospy.loginfo("\033[32m serial open success...1\033[0m")
        else:
            wt_imu.open()
            rospy.loginfo("\033[32m serial open success...2\033[0m")
    except Exception as e:
        print(e)
        rospy.loginfo("\033[31m serial open fail\033[0m")
        exit(0)

    imu_pub = rospy.Publisher("wit/imu", Imu, queue_size=10)

    while not rospy.is_shutdown():
        buff_count = wt_imu.inWaiting()
        if buff_count > 0 and iapflag == 0:
            buff_data = wt_imu.read(buff_count)
            if recordflag:
                recordbuff = recordbuff + buff_data
            for i in range(0, buff_count):
                handleSerialData(buff_data[i])
        r.sleep()

    rospy.spin()
