#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import rospy
from robot_msgs.msg import messages, hardware
from armSchedule import Schedule
from config import jointParam, jointReachThreshold, switchMap, gripperMap

class Movement():

    def __init__(self):

        self.msgPub = rospy.Publisher('message', messages, queue_size=3)
        self.ctlPub = rospy.Publisher('control', hardware, queue_size=3)

        self.messages = messages()
        self.move = hardware()
        self.ioMap = {'lower_left':3, 'lower_right':1, 'upper_left':0, 'upper_right':2}
        self.armID = ['DL', 'DR', 'UL', 'UR']
        self._pass = {'lower_left': False, 'lower_right': False, 'upper_left': False, 'upper_right': False}

        self.gripper = {'lower_left': [200, 1650], 'lower_right': [1600, 300], 'upper_left': [50, 1350], 'upper_right': [1800, 400]}
        self.IDX = {'lower_left': 0, 'lower_right': 1, 'upper_left': 2, 'upper_right': 3}
        self.motors = Schedule()

    def origin(self):
        self.messages.action = 'origining'
        self.msgPub.publish(self.messages)
        rospy.sleep(0.2)
        # 发布控制信息
        self.ctlPub.publish(self.move)
        # 设定期望位置
        ArmMotorDesiredPosition = np.array([-0, 0, -0, 0, 0, -0, 0, 0, 0, 0])
        # 获取当前位置
        ArmMotorCurrentPosition = self.motors.currentPosition
        # 初始化计数器
        count = np.zeros(10)
        # 当程序没有被关闭时
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            # 更新当前位置
            ArmMotorCurrentPosition = self.motors.currentPosition
            # 计算当前位置与期望位置的差值
            delta = ArmMotorCurrentPosition - ArmMotorDesiredPosition
            # 根据差值调整速度
            for i in range(10):
                if np.abs(delta[i]) > 0.005:
                    self.move.target_velocity[i] = int(single_adaptive(delta[i], i))
                else:
                    count[i] = 1
                    self.move.target_velocity[i] = 0
            # 发布控制信息
            self.ctlPub.publish(self.move)
            # 若所有轴都满足条件，退出循环
            if np.sum(count) == 10:
                self.move.target_velocity = [0 for i in range(10)]
                self.move.switch_open = []
                self.move.switch_close = [0, 1, 2, 3]
                self.ctlPub.publish(self.move)
                break
            rospy.sleep(0.5)
        self.messages.action = 'origined'
        self.msgPub.publish(self.messages)

    def observation(self):
        # 将四个机械臂移动至固定构型，来获取全局果实分布
        self.messages.action = 'observing'
        self.msgPub.publish(self.messages)
        rospy.sleep(0.2)
        ArmMotorDesiredPosition = np.array([-0.3, 0.1, -0.54, 0.54, 0.54, -0.54, 0, 0, 0, 0])
        ArmMotorCurrentPosition = self.motors.currentPosition
        # 若到达指定位置，isReached 元素赋为1
        count = np.zeros(10)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            ArmMotorCurrentPosition = self.motors.currentPosition
            delta = ArmMotorCurrentPosition - ArmMotorDesiredPosition
            for i in range(10):
                if np.abs(delta[i]) > 0.005:
                    self.move.target_velocity[i] = int(single_adaptive(delta[i], i))
                else:
                    count[i] = 1
                    self.move.target_velocity[i] = 0
            self.ctlPub.publish(self.move)
            # 满足条件，退出
            if np.sum(count) == 10:
                self.move.target_velocity = [0 for i in range(10)]
                self.move.switch_open = []
                self.move.switch_close = [0, 1, 2, 3]
                self.ctlPub.publish(self.move)
                break
        self.messages.action = 'observed'
        self.msgPub.publish(self.messages)

    def approach(self, target, jointV, jointH, jointR):
        # 现在新加了旋转方向
        v = target[0]
        h = target[1]
        r = target[3]
        self.motors.desirePosition[jointV] = v
        self.motors.desirePosition[jointH] = h
        self.motors.desirePosition[jointR] = r
        rospy.sleep(0.2)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            deltaVertical = np.abs(v - self.motors.currentPosition[jointV])
            deltaHorizontal = np.abs(h - self.motors.currentPosition[jointH])
            deltaHorizontal = np.abs(h - self.motors.currentPosition[jointR])
            if deltaVertical < jointReachThreshold['vertical'] and deltaHorizontal < jointReachThreshold['horizontal']: break


    def pick(self, unit, joint, target):
        '''
        机械臂伸出和抓取动作
        - unit：作业单元，即机械臂；
        - joint：此次动作需要运动的关节编号；
        - target：各关节运动目标位置。
        '''
        e = target[2]
        # 气缸伸出
        self.cylinderExtend(switchMap[unit])
        # 模组伸出
        self.motors.desirePosition[joint] = e
        rospy.sleep(2)
        # 等待直到各关节到达目标位置
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            deltaExtend = np.abs(e - self.motors.currentPosition[joint])
            if deltaExtend < jointReachThreshold['extend']:
                break
        # 机械臂抓取


        self.grasp(gripperMap[unit][4], gripperMap[unit][0])
        rospy.sleep(1.5)
        self.roll(gripperMap[unit][4], gripperMap[unit][2])
        rospy.sleep(2)


    # 换一个函数名称，保留之前的函数。

    def place(self, unit, jointE, jointH):
        '''
        机械臂伸出和抓取动作
        - unit：作业单元，即机械臂；
        - jointE：伸缩运动关节编号；
        - jointH：水平运动关节编号；
        - target：各关节运动目标位置。
        '''
        self.cylinderRetract(switchMap[unit])
        self.motors.desirePosition[jointE] = 0
        self.motors.desirePosition[jointH] = 0
        # 不确定是否需要调整旋转
        rospy.sleep(2)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            deltaExtend = np.abs(0 - self.motors.currentPosition[jointE])
            deltaHorizontal = np.abs(0 - self.motors.currentPosition[jointH])
            if deltaExtend <  jointReachThreshold['extend'] and deltaHorizontal < jointReachThreshold['horizontal']:
                break

        # grasp抓取函数 

        self.grasp(gripperMap[unit][4], gripperMap[unit][1])
        rospy.sleep(1.5)
        self.roll(gripperMap[unit][4], gripperMap[unit][3])
        rospy.sleep(2)



    # 新的非实时命令换成字符串
    def cylinderExtend(self, agent):
        self.move.cylinder = agent + '_extend'
        self.ctlPub.publish(self.move)

    def cylinderRetract(self, agent):
        self.move.cylinder = agent + '_retract'
        self.ctlPub.publish(self.move)

    def gripperRoll(self, agent):
        self.move.gripper = agent + '_roll'
        self.ctlPub.publish(self.move)

    def gripperGrasp(self, agent):
        self.move.gripper = agent + '_grasp'
        self.ctlPub.publish(self.move)

    def roll(self, unit, t):
        val = self.gripper[unit][0] + 100 * t
        self.gripperOpt.gripper_operation[self.IDX[unit] + 4] = val
        self.gripperPub.publish(self.gripperOpt)

    def back(self, unit):
        val = self.gripper[unit][1]
        self.gripperOpt.gripper_operation[self.IDX[unit] + 4] = val
        self.gripperPub.publish(self.gripperOpt)  







