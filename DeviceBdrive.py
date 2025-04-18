#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ctypes import *
import rospy
import numpy as np
from robot_msg.msg import hardware

'''
Can控制器B的驱动，负责两个手爪的控制。
'''

class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_int32),
                ("AccMask", c_int32),
                ("Reserved", c_int32),
                ("Filter", c_ubyte),
                ("Timing0", c_ubyte),
                ("Timing1", c_ubyte),
                ("Mode", c_ubyte)]


class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint),
                ("TimeStamp", c_int),
                ("TimeFlag", c_byte),
                ("SendType", c_byte),
                ("RemoteFlag", c_byte),
                ("ExternFlag", c_byte),
                ("DataLen", c_byte),
                ("Data", c_ubyte * 8),
                ("Reserved", c_byte * 3)]
    
VCI_USBCAN2 = 4

ubyte_array = c_ubyte*8
data = ubyte_array(0, 0, 0, 0, 0, 0, 0, 0)


# def callback(msgs):
#     if msgs.

rospy.init_node('robot_driveB')

rospy.subscriber("gripper_control", hardware, callback)


try:
    CANalystII = CDLL("./libcontrolcan.so")
    print("Loaded CANalystII library")
except OSError as e:
    CANalystII = None
    print("Cannot load CANalystII library")


pInfo = CANalystII.VCI_FindUsbDevice2(50)

for i in range(50):
    if pInfo[i].str_Serial_Num == 555:
        device = i
        break

print(device)

# 打开设备
ret = CANalystII.VCI_OpenDevice(VCI_USBCAN2, device, 0)
if ret == STATUS_OK:
    print('调用 VCI_OpenDevice成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_OpenDevice出错\r\n')


# 初始化CAN配置
vci_initconfig = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0,
                                 0, 0x00, 0x1C, 0)#波特率500k，正常模式

#初始0通道
ret = CANalystII.VCI_InitCAN(VCI_USBCAN2, device, 0, byref(vci_initconfig))
if ret == STATUS_OK:
    print('调用 VCI_InitCAN1成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_InitCAN1出错\r\n')

ret = CANalystII.VCI_StartCAN(VCI_USBCAN2, device, 0)
if ret == STATUS_OK:
    print('调用 VCI_StartCAN1成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_StartCAN1出错\r\n')

#初始1通道
ret = CANalystII.VCI_InitCAN(VCI_USBCAN2, device, 1, byref(vci_initconfig))
if ret == STATUS_OK:
    print('调用 VCI_InitCAN2 成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_InitCAN2 出错\r\n')
 
ret = CANalystII.VCI_StartCAN(VCI_USBCAN2, device, 1)
if ret == STATUS_OK:
    print('调用 VCI_StartCAN2 成功\r\n')
if ret != STATUS_OK:
    print('调用 VCI_StartCAN2 出错\r\n')


# 循环发送和接收数据
while not rospy.is_shutdown():

    res = CANalystII.VCI_InitDevice(device)





