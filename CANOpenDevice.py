#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import canopen


class ServoMotor():

    def __init__(self, num):
        self.node = canopen.RemoteNode(num, '/home/arl/HarvestRobot/src/robot_drive/EDS/KINCO-JD-FD.eds')# eds file path
        self.target_velocity = 'Target_velocity'
        self.actual_position = 'Position_actual_value'
        self.statusWord = 'Statusword'
        self.status = 0
        self.node_id = num
        # bit0：准备上电 bit1：已上电 bit2：使能 bit3：故障 bit4：禁止输出电压 bit5：快速停止 
        # bit6：上电禁止 bit7：警告 bit8：内部保留 bit9：远程控制 bit10：目标位置到 bit11：内部限位激活 bit12：脉冲响应 
        # bit13：跟随误差/原点错误 bit14：找到电机励磁 bit15：原点找到
        self.StateDict = ['Ready', 'PowerOn', 'Enable', 'Error', 'OutputForbidden', 'QuickStop', 'PowerOnForbidden','Warning', 
                     'Inner reserve', 'Remote Control', 'TargetPositionTo', 'Active inner position limit', 'Pulse response', 
                     'TrackingError/Origin Error', 'Motor Excitation Found','OriginFound']


    # 配置pdo  
    def pdo_mapping(self):
        # print('node {} is mapping'.format(str(self.node_id)))
        self.node.tpdo.read()
        self.node.rpdo.read()

        self.node.tpdo[1].clear()
        self.node.tpdo[1].cob_id = 384 + self.node_id
        self.node.tpdo[1].add_variable('Statusword')
        self.node.tpdo[1].add_variable('Position_actual_value')
        self.node.tpdo[1].trans_type = 254
        # self.node.tpdo[1].event_timer = 10
        self.node.tpdo[1].enable = True

        self.node.tpdo[2].clear()
        self.node.tpdo[2].cob_id = 640 + self.node_id
        self.node.tpdo[2].add_variable('Error Status')
        self.node.tpdo[2].add_variable('Digital In and Output','Input Status')
        self.node.tpdo[2].trans_type = 254
        # self.node.tpdo[1].event_timer = 10
        self.node.tpdo[2].enable = True

        self.node.rpdo[1].clear()
        self.node.rpdo[1].cob_id = 512 + self.node_id
        self.node.rpdo[1].add_variable('Modes_of_operation')
        self.node.rpdo[1].add_variable('Controlword')
        self.node.rpdo[1].add_variable('Target_velocity')
        self.node.rpdo[1].trans_type = 255
        self.node.rpdo[1].enable = True

        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.tpdo.save()
        self.node.rpdo.save()
        # self.node.nmt.state = 'OPERATIONAL'

    # 定义运行模式
    def mode(self, mode=3):
        # 3 速度模式， 6 回原点模式
        self.node.rpdo[1]['Modes_of_operation'].raw = mode
        # self.node.sdo['Modes_of_operation'].raw = mode

    def setCommand(self, word):
        self.node.rpdo[1]['Controlword'].raw = word

    # 电机上电
    def on(self):
        self.node.rpdo[1]['Controlword'].raw = 15
        # self.node.sdo['Controlword'].raw = 15
        print('motor{} on'.format(str(self.node_id)))

    # 电机断电
    def off(self):
        self.node.rpdo[1]['Controlword'].raw = 6
        # self.node.sdo['Controlword'].raw = 6  
        print('motor{} off'.format(str(self.node_id)))

    # 电机运行
    def run(self, data):
        self.node.rpdo[1][self.target_velocity].phys = data
        # self.node.sdo[self.md].phys = data

    # 电机位置
    def getPulse(self): 
        # N*A/B = s*n*r/P,
        # where A/B denote shear ratios; N is pulse numer; s is placement of mechanism
        # n (1:n) is speed ratio; r is pulse count per round; P (Unit:mm) srew distance   
        # Get pulse count N. 
        # reture the pulse count N
        PulseCount = self.node.tpdo[1][self.actual_position].phys
        return PulseCount
        # return self.node.sdo['Position_actual_value'].phys

    def getStatus(self):
        _state = self.node.tpdo[1]['Statusword'].raw
        _error = self.node.tpdo[2]['Error Status'].raw
        _limit = self.node.tpdo[2]['Digital In and Output.Input Status'].raw
        return [_state, _error, _limit]

    def getIO(self):
        _limit = self.node.tpdo[2]['Digital In and Output.Input Status'].raw
        return bin(_limit)[2:].zfill(16)

    def getState(self):
        _state = self.node.tpdo[1]['Statusword'].raw
        return bin(_state)[2:].zfill(16)
    
    def getError(self):
        # bit0：内部错误报警 
        # bit 1：编码器 ABZ 连接报警 
        # bit 2：编码器 UVW 连接报警 
        # bit 3：编码器计数报警 
        # bit 4：驱动器高温报警 
        # bit 5：驱动器高压报警 
        # bit 6：驱动器低压报警 
        # bit 7：驱动器过流报警 
        # bit 8：吸收电阻报警 
        # bit 9：位置误差过大报警  
        # bit 10：逻辑低压报警 
        # bit 11：电机或驱动器 iit 报警 
        # bit 12：脉冲频率过高报警  
        # bit 13：电机高温报警  
        # bit 14：电机励磁报警  
        # bit 15：存储器报警
        _error = self.node.tpdo[2]['Error Status'].raw
        return bin(_error)[2:].zfill(16)    

    # Start PDO transmision
    def rpdo_start(self, rate):
        self.node.rpdo[1]['Modes_of_operation'].raw = 3
        self.node.rpdo[1]['Controlword'].raw = 6
        self.node.rpdo[1]['Target_velocity'].phys = 0
        self.node.rpdo[1].start(rate)
        # return __done


    # Stop PDO transmision
    def rpdo_stop(self):
        self.node.rpdo[1].stop()

    # 回零点
    def zero_back(self):
        # 1：带位置环的定位模式 3：带位置环的速度模式 4：力矩模式 -3：速度环(立即速度模式) -4：脉冲模式 6：找原点模式 7：基于 CANopen 的运动插补
        # switch to origin found 
        self.mode(6)
        self.node.rpdo[1]['Controlword'].raw = 31




# 手爪透传模块
class Gripper():
    def __init__(self, num):
        self.node = canopen.RemoteNode(num, '/home/arl/HarvestRobot/src/robot_drive/EDS/GCAN400.eds')# eds file path
        self.node_id = num
        # self.targetGrip = 0
        # self.targetRoll = 0
        # self.checkGrip = 0
        # self.checkRoll = 0

    # 配置pdo  
    def pdo_mapping(self):
        self.node.tpdo[1].clear()
        self.node.tpdo[1].cob_id = 384 + self.node_id
        self.node.tpdo[1].add_variable('1. Transmit PDO mapping parameter', 'PDO mapping 1. app. object')
        self.node.tpdo[1].add_variable('1. Transmit PDO mapping parameter', 'PDO mapping 2. app. object')
        self.node.tpdo[1].trans_type = 254
        self.node.tpdo[1].event_timer = 50
        self.node.tpdo[1].enabled = True

        self.node.rpdo[1].clear()
        self.node.rpdo[1].cob_id = 512 + self.node_id
        self.node.rpdo[1].add_variable('1. Receive PDO mapping parameter', 'PDO mapping 1. app. object')
        self.node.rpdo[1].add_variable('1. Receive PDO mapping parameter', 'PDO mapping 2. app. object')
        self.node.rpdo[1].enabled = True

        self.node.rpdo[2].clear()
        self.node.rpdo[2].cob_id = 768 + self.node_id
        self.node.rpdo[2].add_variable('2. Receive PDO mapping parameter', 'PDO mapping 1. app. object')
        self.node.rpdo[2].add_variable('2. Receive PDO mapping parameter', 'PDO mapping 2. app. object')
        self.node.rpdo[2].enabled = True

        self.node.rpdo[3].clear()
        self.node.rpdo[3].cob_id = 1024 + self.node_id
        self.node.rpdo[3].add_variable('3. Receive PDO mapping parameter', 'PDO mapping 1. app. object')
        self.node.rpdo[3].add_variable('3. Receive PDO mapping parameter', 'PDO mapping 2. app. object')
        self.node.rpdo[3].enabled = True

        self.node.rpdo[4].clear()
        self.node.rpdo[4].cob_id = 1280 + self.node_id
        self.node.rpdo[4].add_variable('4. Receive PDO mapping parameter', 'PDO mapping 1. app. object')
        self.node.rpdo[4].add_variable('4. Receive PDO mapping parameter', 'PDO mapping 2. app. object')
        self.node.rpdo[4].enabled = True

        self.node.nmt.state = 'PRE-OPERATIONAL'
    
    def grip(self, val):
        target, check = tools.grip_calculate(val)
        self.node.rpdo[3]['3. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\xFF\xFF\x01\x09'
        self.node.rpdo[3]['3. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = b'\x03\x2A'+target
        self.node.rpdo[4]['4. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\x00\x00\xE8\x03'
        self.node.rpdo[4]['4. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = check + b'\x00\x00\x00'

    def roll(self, val):
        target, check = tools.row_calculate(val)
        self.node.rpdo[3]['3. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\xFF\xFF\x02\x09'
        self.node.rpdo[3]['3. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = b'\x03\x2A'+target
        self.node.rpdo[4]['4. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\x00\x00\xE8\x03'
        self.node.rpdo[4]['4. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = check + b'\x00\x00\x00'


    def operate(self, gripVal, rollVal):
        target1, target2, check = tools.sync_calculate(gripVal, rollVal)
        self.node.rpdo[2]['2. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\xFF\xFF\xFE\x12'
        self.node.rpdo[2]['2. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = b'\x83\x2A\x06\x01'
        self.node.rpdo[3]['3. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = target1 + b'\x00\x00'
        self.node.rpdo[3]['3. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = b'\xE8\x03\x02'+target2[0]
        self.node.rpdo[4]['4. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = target2[1]+b'\x00\x00\xE8'
        self.node.rpdo[4]['4. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = b'\x03'+check+b'\x00\x00'

    def posGet(self):
        self.node.rpdo[1]['1. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\xFF\xFF\x01\x04'
        self.node.rpdo[1]['1. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = b'\x02\x38\x03\xBD'

    def grip_currentGet(self):
        self.node.rpdo[1]['1. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\xFF\xFF\x01\x04'
        self.node.rpdo[1]['1. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = b'\x02\x45\x02\xB1'

    def row_currentGet(self):
        self.node.rpdo[1]['1. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\xFF\xFF\x02\x04'
        self.node.rpdo[1]['1. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = b'\x02\x45\x02\xB0'

    # def torqueHold(self):
    #     self.node.rpdo[4]['4. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\xFF\xFF\x01\x04'
    #     self.node.rpdo[4]['4. Receive PDO mapping parameter.PDO mapping 2. app. object'].raw = b'\x03\x28\x01\xCE'

    # Start PDO transmision
    def rpdo_start(self, rateSend, rateRecv):
        self.node.rpdo[1].start(rateRecv)
        self.node.rpdo[2].start(rateSend)
        self.node.rpdo[3].start(rateSend)
        self.node.rpdo[4].start(rateSend)

    # Stop PDO transmision
    def rpdo_stop(self):
        self.node.rpdo[1].stop()
        self.node.rpdo[2].stop()
        self.node.rpdo[3].stop()
        self.node.rpdo[4].stop()


# 继电器
class RelayIO():
    def __init__(self, num):
        self.node = canopen.RemoteNode(num, '/home/arl/HarvestRobot/src/robot_drive/EDS/switch.eds')# eds file path
        self.node_id = num
        self.io = ['0' for i in range(8)]
        # dictionary
        # self.pdo_mapping()

    def pdo_mapping(self):
        self.node.tpdo[1].clear()
        self.node.tpdo[1].cob_id = 384 + self.node_id
        self.node.tpdo[1].add_variable('1. Transmit PDO mapping parameter', 'PDO mapping 1. app. object')
        self.node.tpdo[1].event_timer = 5
        self.node.tpdo[1].enabled = True

        self.node.rpdo[1].clear()
        self.node.rpdo[1].cob_id = 512 + self.node_id
        self.node.rpdo[1].add_variable('1. Receive PDO mapping parameter', 'PDO mapping 1. app. object')
        self.node.rpdo[1].enabled = True

        self.node.nmt.state = 'PRE-OPERATIONAL'
    
    def Open(self, bit):
        # INPUT: should be a list
        # OUTPUT: a byte  
        for i in list(bit):
            self.io[i] = '1'
        buf = bytes([int(''.join(self.io), 2)])
        self.node.rpdo[1]['1. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = buf
        # open check 

    def Close(self, bit):
        # INPUT: should be a list
        # OUTPUT: a byte  
        for i in list(bit):
            self.io[i] = '0'
        buf = bytes([int(''.join(self.io), 2)])
        self.node.rpdo[1]['1. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = buf
        # open check 


    def allClose(self):
        self.node.rpdo[1]['1. Receive PDO mapping parameter.PDO mapping 1. app. object'].raw = b'\x00'

    # Start PDO transmision
    def rpdo_start(self, rate):
        self.node.rpdo[1].start(rate)

    # Stop PDO transmision
    def rpdo_stop(self):
        self.node.rpdo[1].stop()