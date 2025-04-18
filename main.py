#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import os,sys
import rospy
import numpy as np
from robot_msgs.msg import robotcore, hardware, messages
import _thread



'''
顶层控制，负责所有功能模块的启动和配置
'''



# class Harvester():
class Harvester():

    def __init__(self):
        rospy.init_node('robot_main')




if __name__ == '__main__':
    try:
        Harvester()
    except rospy.ROSInterruptException:
        pass







