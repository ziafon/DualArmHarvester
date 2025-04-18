#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from robot_msgs.msg import messages
import _thread
from movement import Movement
from config import jointMap
from utils import invCalculate, tf_base2fake_listener
from config import jointMap, armName, stretchDIR

'''
调度功能的总入口，负责机器人所有行为的执行。
通过ROS话题指令执行相应的行为，包括回原点，去观测位，以及机械臂采摘行为等。
其中为每个目标采摘流程开辟一个线程，目标采摘结束后线程自动销毁。

'''

class Schedule():

    def __init__(self):
         
        rospy.init_node('robot_control')
        self.movement = Movement()
        self.tf_linsener = tf_base2fake_listener(armName)
        self.canExtend = {'lower_left':True, 'lower_right':True, 'upper_left':True, 'upper_right':True}
        self.pub = rospy.Publisher('robot_feedback', messages, queue_size=3)
        rospy.Subscriber('robot_command', messages, callback=self.Callback, queue_size=3)
        rospy.spin()


    def Callback(self, msgs):
        if msgs.command == 'observation':
            self.movement.observation()
        if msgs.command == 'origin':
            self.movement.origin()
        if msgs.command == 'harvest':
            agent = msgs.agent
            target = msgs.target
            # 手臂编号确定
            i = 0 if agent == 'lower' else 1
            joint = jointMap[agent]
            # 坐标转换，生成关节位移
            tf_listener = self.tf_linsener[i]
            id = armName[i]
            direction = stretchDIR[agent]
            joint_desire_position = invCalculate(target, tf_listener, id, direction)
            _thread.start_new_thread(self.harvestPipeline, ([agent, joint, joint_desire_position]))


    def harvestPipeline(self, agent, joint, position):
        jointHorizontal = joint[1]
        jointExtend = joint[2]
        self.movement.approach(position, joint[0], jointHorizontal)
        while self.canExtend[agent] == False:
            rospy.sleep(0.1)
        self.movement.pick(agent, jointExtend, position)
        self.canExtend[agent] = False
        call4next = messages()
        call4next.workingState = 'picked'
        call4next.agent = agent
        self.pub.publish(call4next)
        self.movement.place(agent)
        self.canExtend[agent] = True
