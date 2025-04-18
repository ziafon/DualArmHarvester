#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from robot_msgs.msg import messages
import _thread
from movement import Movement
from config import jointMap
import _thread
from task import simple_plan


class Plan():
    def __init__(self):
        rospy.init_node('robot_plan')
        self.mainPub = rospy.Publisher('command', messages, queue_size=3)
        self.Targets = {'lower':[], 'upper': []}
        self.pointer = {'lower':0, 'upper': 0}

        rospy.Subscriber('command', messages, callback=self.mainCallback)
        rospy.spin()


    '''
    plan.current 和plan.next 分别取各个agent的当前坐标和下一个坐标
    plan.pointer 记录了每个agent任务列表的指针位置
    plan.Targets 记录了所有agent需要到达的坐标的列表

    '''

    def current(self, agent):
        coord1, coord2, coord3 = self.Targets[agent][self.pointer[agent] - 1]
        return [coord1, coord2, coord3]
    
    def next(self, agent):
        coord1, coord2, coord3 = self.Targets[agent][self.pointer[agent]]
        return [coord1, coord2, coord3]

    def update(self, agent):
        if self.pointer[agent] < len(self.Targets[agent]):
            self.pointer[agent] += 1
        elif self.pointer[agent] == len(self.Targets[agent]):
            self.Targets[agent].append([-1, -1, -1])

    def reset(self, agent):
        self.pointer[agent] = 0

    '''
    plan.state 表示当前计划的状态，有三种状态：Done,Planning,Error
    '''

    def state(self, agent):
        if self.pointer[agent] == len(self.Targets[agent]):
            return 'Done'
        else:
            return 'Planning'
    def count(self, agent):
        num_picked = self.pointer[agent] 
        num_nonPicked = len(self.Targets[agent]) -  num_picked
        return num_picked, num_nonPicked
    def Error(self, agent):
        if self.pointer[agent] > len(self.Targets[agent]):
            return 'Pointer is larger than list length'
        elif self.pointer[agent] <= 0:
            return 'Invalid Pointer'


    def mainCallback(self, msgs):
        if msgs.command == 'recieve_apple_distribution':
            objects = msgs.objects
            self.Targets = simple_plan(np.array(objects).reshape(-1, 3), self.Targets)
        if msgs.command == 'excecution':
            for agent in ['lower', 'upper']:
                self.reset(agent)
                self.targetPublish(agent)
        if msgs.workingState == 'picked':
            agent = msgs.agent
            self.update(agent)
            self.targetPublish(agent)


    def targetPublish(self, agent):
        target = self.Targets[agent][self.pointer[agent]]
        currentTask = messages()
        currentTask.command = 'harvest'
        currentTask.target = target[0:3]
        currentTask.agent = agent
        currentTask.arm = 'left' if target[3] == 0 else 'right'
        self.mainPub.publish(currentTask)





