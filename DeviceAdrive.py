#!/usr/bin/env python3
# -*- coding: utf-8 -*-


'''
Can控制器A的驱动，负责电机和继电器的控制。
'''


import rospy
from robot_msgs.msg import hardware
import canopen
import _thread
from CANOpenDevice import ServoMotor
from CANDevice import Gripper, Switch
from config import jointParam, initData, switchBit, switchFlag


class Drive():
    # 八个电机，ID从21到28，但是目前只有6个电机，锁定了最后两个旋转。
    def __init__(self):
        rospy.init_node('robot_driveA')
        self.jointNum = jointParam['jointNum']
        self.network = canopen.Network()
        self.servomotors = [ServoMotor(i+21) for i in range(self.jointNum)]
        self.lower_gripper = Gripper(0x99)
        self.upper_gripper = Gripper(0x100)
        self.switch = Switch(0x101)
        self.canMessages = {'lower': self.lower_gripper.default(), 
                            'upper': self.upper_gripper.default(), 
                            'switch': self.switch.default()}
        self.devices_init()
        self.rpdo_open()
        self.pub = rospy.Publisher('robot_feedback', hardware, queue_size=10)
        _thread.start_new_thread(self.canTransmit,())
        _thread.start_new_thread(self.feedback_publish,())
        rospy.Subscriber('robot_control', hardware, self.callback, queue_size=20)
        rospy.spin()


    def callback(self, msgs):
        if msgs.target_velocity:
            for i in range(self.jointNum):
                self.servomotors[i].run(msgs.target_velocity[i])
        if msgs.motor_mode:
            for i in range(self.jointNum):
                self.servomotors[i].mode(msgs.motor_mode[i])
        if msgs.motor_control:
            for i in range(self.jointNum):
                self.servomotors[i].setCommand(msgs.motor_control[i])
        if msgs.gripper_operation:
            opt = msgs.gripper_operation
            agent = msgs.agent
            if agent == 'lower':
                self.canMessages[agent] = self.lower_gripper.operation(opt)
            if agent == 'upper':
                self.canMessages[agent] = self.upper_gripper.operation(opt)
            # print(self.canMessages[agent])
        if msgs.switch_operation:
            tmp = msgs.switch_operation
            dvice, opt = tmp.split('_')
            bit, flag = switchBit[dvice], switchFlag[opt]
            self.canMessages['switch'] = self.switch.operation(bit, flag)

        
    def feedback_publish(self):
        fb = hardware()
        status = ['0000000000000000' for i in range(self.jointNum)]
        limit = ['0000000000000000' for i in range(self.jointNum)]
        error = ['0000000000000000' for i in range(self.jointNum)]
        position = [0 for i in range(self.jointNum)]
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            for i in range(self.jointNum):
                position[i] = self.servomotors[i].getPulse()
                status[i] = self.servomotors[i].getState()
                limit[i] = self.servomotors[i].getIO()
            fb.actual_position = position
            fb.motor_status = status
            fb.io_status = limit
            fb.error_status = error
            self.pub.publish(fb)


    def devices_init(self):
        # add nodes
        for i in range(self.jointNum):
            self.network.add_node(self.servomotors[i].node)
        # connect from CAN bus
        self.network.connect(bustype="canalystii", channel=[0,1], bitrate=500000)
        # 若连接失败，捕获异常，并返回。
        print('已连接')
        try:
            for i in range(self.jointNum):
                self.servomotors[i].pdo_mapping()
        except Exception as e:
            print('\033[1;31m{} 总线连接失败！！！\033[0m\n'.format('xie'))
            return
        self.network.sync.start(0.1)
        for i in range(self.jointNum):
            self.servomotors[i].node.nmt.state = 'OPERATIONAL'
        rospy.sleep(1)
        rospy.loginfo('所有设备初始化成功 ！！')


    def rpdo_open(self):
        for i in range(self.jointNum):
            self.servomotors[i].rpdo_start(0.1)

    def rpdo_close(self):
        for i in range(self.jointNum):
            self.servomotors[i].rpdo_stop()
        # self.grippers.rpdo_stop()

    def devices_down(self):
        # Disconnect from CAN bus
        self.rpdo_close()
        self.network.sync.stop()
        self.network.disconnect()

    def canTransmit(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.25)
            for key in self.canMessages:
                self.network.bus.send(self.canMessages[key])


if __name__=='__main__': 
    try:
        Drive()
    except KeyboardInterrupt:
        rospy.loginfo('Hasta la Vista...')
            

