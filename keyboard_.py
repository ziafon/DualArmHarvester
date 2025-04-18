#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
# from pynput.keyboard import Key,Listener
from robot_msgs.msg import hardware

pub = rospy.Publisher('robot_control', hardware, queue_size=10)
msg = hardware()
vec = [0 for i in range(8)]
arm = 0
# flag = [[1,5,9], [1,4,8], [0,2,6], [0,3,7]]  
# flag = [[7,0], [4,3], [5,2], [6,1]]
flag = [[1, 3, 5, 6], [0, 2, 4, 7]]



def on_press(key):
    global vec, pub, arm, flag, msg
    try:
        if key.char=='w': 
            vec[flag[arm][0]] = 500000
            msg.target_velocity = vec
            pub.publish(msg)
        elif key.char=='s': 
            vec[flag[arm][0]] = -500000
            msg.target_velocity = vec
            pub.publish(msg)
        if key.char=='a': 
            vec[flag[arm][1]] = -500000
            msg.target_velocity = vec
            pub.publish(msg)
        elif key.char=='d': 
            vec[flag[arm][1]] = 500000
            msg.target_velocity = vec
            pub.publish(msg)
        if key.char=='j': 
            vec[flag[arm][2]] = 500000
            msg.target_velocity = vec
            pub.publish(msg)
        elif key.char=='l': 
            vec[flag[arm][2]] = -500000
            msg.target_velocity = vec
            pub.publish(msg)
        if key.char=='i': 
            vec[flag[arm][3]] = 200000
            msg.target_velocity = vec
            pub.publish(msg)
        elif key.char=='k': 
            vec[flag[arm][3]] = -200000
            msg.target_velocity = vec
            pub.publish(msg)
    except AttributeError:
        pass



def on_release(key):
    global vec, pub, arm, flag, msg
    try:
        if key.char=='w': 
            vec[flag[arm][0]] = 0
            msg.target_velocity = vec
            pub.publish(msg)
        elif key.char=='s': 
            vec[flag[arm][0]] = 0
            msg.target_velocity = vec
            pub.publish(msg)
        if key.char=='a': 
            vec[flag[arm][1]] = 0
            msg.target_velocity = vec
            pub.publish(msg)
        elif key.char=='d': 
            vec[flag[arm][1]] = 0
            msg.target_velocity = vec
            pub.publish(msg)
        if key.char=='j': 
            vec[flag[arm][2]] = 0
            msg.target_velocity = vec
            pub.publish(msg)
        elif key.char=='l': 
            vec[flag[arm][2]] = 0
            msg.target_velocity = vec
            pub.publish(msg)
        if key.char=='i': 
            vec[flag[arm][3]] = 0
            msg.target_velocity = vec
            pub.publish(msg)
        elif key.char=='k': 
            vec[flag[arm][3]] = 0
            msg.target_velocity = vec
            pub.publish(msg)
        if key.char=='v':
            arm=0
            print ('下')
        elif key.char=='b':
            arm=1
            print ('上')
        if key.char=='q':
            print ('exit keyboard mode')
            return False
    except AttributeError:
        pass