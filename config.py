#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

_JointName = ['vertical_motion_lower_arm_joint',
                    'horizontal_motion_lower_left_arm_joint',
                    'stretch1_motion_lower_left_arm_joint',
                    'stretch2_motion_lower_left_arm_joint',
                    'horizontal_motion_lower_right_arm_joint',
                    'stretch1_motion_lower_right_arm_joint',
                    'stretch2_motion_lower_right_arm_joint',
                    'vertical_motion_upper_arm_joint',
                    'horizontal_motion_upper_left_arm_joint',
                    'stretch1_motion_upper_left_arm_joint',
                    'stretch2_motion_upper_left_arm_joint',
                    'horizontal_motion_upper_right_arm_joint',
                    'stretch1_motion_upper_right_arm_joint',
                    'stretch2_motion_upper_right_arm_joint']

jointParam = {'jointNum': 8, 'urdfJointNum': 10}

motorParam = {'vertical': 10000*50/0.18, 'horizontal': 10000*10/0.18, 'extend': 10000*10/0.16}

jointMap = {'lower':[1, 3, 6, 5], 'upper':[0, 2, 7, 4]}

switchMap =  {'lower_left':3, 'lower_right':1, 'upper_left':0, 'upper_right':2}

stretchDIR =  {'lower':-1, 'upper':1}

jointReachThreshold = {'vertical': 0.015, 'horizontal': 0.01, 'extend': 0.01}

gripperData = {'grasp':   [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            'release': [0x00, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            'rollPos': [0x00, 0x00, 0x00, 0x00, 0x3F, 0x2F, 0x00, 0x00],
            'rollNeg': [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            }

initData =  {'gripper': [0x00, 0x6F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 
             'switch': [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]}

switchBit = {'lower':0, 'upper':1, 'light':2}
switchFlag = {'extend':True, 'retract':False, 'on':True, 'off': False}