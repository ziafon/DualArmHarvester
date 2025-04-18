#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import os,sys
import rospy
import numpy as np
from robot_msgs.msg import system, hardware
from sensor_msgs.msg import JointState
import _thread
from tools import distance_calculate, convert, adaptive, whetherReached, nowTime, single_adaptive, _JointName

'''
机器人控制模块，只负责关节运动控制、气缸的伸缩以及冲突时的避障操作。
'''

class Control():
    def __init__(self):
        rospy.init_node('robot_control')
        self.jointNum = 10
        self.jsPosition = [0.0 for i in range(14)]
        self.currentPosition = np.zeros(self.jointNum)
        self.desirePosition = np.zeros(self.jointNum)
        self.isReached = np.bool_(self.jointNum)
        self.control_publisher = rospy.Publisher('robot_control', hardware, queue_size=20)
        _thread.start_new_thread(self.robotFeedback,())
        _thread.start_new_thread(self.commandSubscribe,())
        _thread.start_new_thread(self.jointSubscribe,())
        self.move = hardware()
        self.move.target_velocity = [0 for i in range(10)]
        self.move.switch_close = [0, 1, 2, 3]
        self.mode = 'WAIT'
        # 定义发布关节消息
        js_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        js = JointState() # 定义机器人关节消息
        js.header.stamp = rospy.Time.now()
        js.name = _JointName
        SAFETHRESHOLD = [0.53, 1.15, 1.15] # 安全阈值
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            js.position = self.joint_positon
            js_pub.publish(js)
            # 回原点
            if self.operation_mode == 'ORIGIN':
                self.origin()
                for i in range(10):
                    self.targetVelocity[i] = 0
            # 观测目标位置
            if self.operation_mode == 'OB':
                self.observation()
                for i in range(10):
                    self.targetVelocity[i] = 0
                self.operation_mode = 'WAIT'
            if self.operation_mode == 'RUN':
                self.isReached = whetherReached(self.currentPosition, self.desirePosition)
                # 计算关键距离，共三个元素。0：上下导轨之间的距离， 1：左下和右下两臂横向距离，2：左上和右上两臂横向距离
                keyDistance = distance_calculate(self.jsPosition)
                # 通过当前位置和期望位置生成自适应速度控制量
                self.targetVelocity = adaptive(self.currentPosition - self.desirePosition) 
                for i in range(3):
                    if keyDistance[i] - SAFETHRESHOLD[i] > -0.05:
                        self.targetVelocity[self.avoid[i]] = 0
                    if keyDistance[i] - SAFETHRESHOLD[i] > 0:
                        self.targetVelocity[self.avoid[i]] = self.targetVelocity[self.priority[i]]
                self.move.target_velocity = np.int32(self.targetVelocity).tolist()
                self.control_publisher.publish(self.move)
  

    def commandSubscribe(self):
        rospy.Subscriber('robot_command', system, callback=self.command_callback, queue_size=3)
        rospy.spin()

    def jointSubscribe(self):
        rospy.Subscriber('position', system, callback=self.joint_callback, queue_size=3)
        rospy.spin()

    def command_callback(self, msgs):
        if msgs.mode:
            self.operation_mode = msgs.mode


    def joint_callback(self, msgs):
        if len(msgs.jointID) > 0 and len(msgs.jointTarget) > 0:
            idx = msgs.jointID
            goal = msgs.jointTarget
            for i in idx:
                self.desirePosition[i] = goal[i]
        if msgs.switch:
            tmp = msgs.switch
            self.cylinder(tmp[0], tmp[1])
        if msgs.priority:
            self.priority = np.array(msgs.priority)
            self.avoid = np.array([1, 9, 5]) - self.priority


    def cylinder(self, id, opt):
        if opt == 0: # 缩回
            if id in self.move.switch_open:
                self.move.switch_open.remove(id)
            self.move.switch_close.append(id)    
        if opt == 1: # 伸出
            if id in self.move.switch_close:
                self.move.switch_close.remove(id)
            self.move.switch_open.append(id)    
        
    def robotFeedback(self):
        rospy.Subscriber('robot_feedback', hardware, callback=self.robot_callback ,queue_size=3)
        rospy.spin()
    
    def robot_callback(self, msgs):
        # print('\033[1;31m{} 警告，驱动节点通信中断！！！\033[0m\n'.format(nowTime()))
        self.gripper_position = msgs.gripper_status[4:8]
        self.gripper_current = msgs.gripper_status[8:12]
        self.motor_position = msgs.actual_position
        self.motor_status = msgs.motor_status
        # print(self.motor_position)
        self.joint_positon, self.currentPosition = convert(self.joint_positon, self.motor_position)
        # 计算关键距离，共三个元素。0：上下导轨之间的距离， 1：左下和右下两臂横向距离，2：左上和右上两臂横向距离
        self.important_distance = distance_calculate(self.joint_positon)  

    def observation(self):
        # 将四个机械臂移动至固定构型，来获取全局果实分布
        # ArmDesiredPosition
        # rospy.loginfo('set to get object position')
        # self.check_isExist = True   
        print(nowTime(), '前往观测位..\n')
        rospy.sleep(0.2)
        ArmMotorDesiredPosition = np.array([-0.3, 0.1, -0.54, 0.54, 0.54, -0.54, 0, 0, 0, 0])
        ArmMotorCurrentPosition = self.currentPosition
        # 若到达指定位置，isReached 元素赋为1
        count = np.zeros(10)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            ArmMotorCurrentPosition = self.currentPosition
            delta = ArmMotorCurrentPosition - ArmMotorDesiredPosition
            for i in range(10):
                if np.abs(delta[i]) > 0.005:
                    self.move.target_velocity[i] = int(single_adaptive(delta[i], i))
                else:
                    count[i] = 1
                    self.move.target_velocity[i] = 0
            self.control_publisher.publish(self.move)
            # 满足条件，退出
            if np.sum(count) == 10:
                self.move.target_velocity = [0 for i in range(10)]
                self.move.switch_open = []
                self.move.switch_close = [0, 1, 2, 3]
                self.control_publisher.publish(self.move)
                break

        rospy.sleep(0.5)
        
        # self.move.gripper_operation = self.gripper_origin
        self.control_publisher.publish(self.move)

        rospy.sleep(0.5)
        print(nowTime(), '到达观测位..\n')
        self.operation_mode = 'READY'


    def origin(self):
        print(nowTime(), '回原点..\n')
        rospy.sleep(0.2)
        # self.move.gripper_operation = self.gripper_origin
        self.control_publisher.publish(self.move)
        ArmMotorDesiredPosition = np.array([-0, 0, -0, 0, 0, -0, 0, 0, 0, 0])
        ArmMotorCurrentPosition = self.currentPosition
        # 若到达指定位置，isReached 元素赋为
        count = np.zeros(10)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            ArmMotorCurrentPosition = self.currentPosition
            delta = ArmMotorCurrentPosition - ArmMotorDesiredPosition
            for i in range(10):
                if np.abs(delta[i]) > 0.005:
                    self.move.target_velocity[i] = int(single_adaptive(delta[i], i))
                else:
                    count[i] = 1
                    self.move.target_velocity[i] = 0
            self.control_publisher.publish(self.move)
            # 满足条件，退出
            # 满足条件，退出
            if np.sum(count) == 10:
                self.move.target_velocity = [0 for i in range(10)]
                self.move.switch_open = []
                self.move.switch_close = [0, 1, 2, 3]
                self.control_publisher.publish(self.move)
                break
        rospy.sleep(0.5)
        print(nowTime(), '到达原点..\n')
        self.operation_mode = 'WAIT'

if __name__=='__main__': 
    try:
        Control()
    except KeyboardInterrupt:
        rospy.loginfo('Hasta la Vista...')