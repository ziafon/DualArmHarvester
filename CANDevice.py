#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import can
import numpy as np
from config import gripperData, initData


class Gripper():

    def __init__(self, ID):
        self.ID = ID
        
    def operation(self, opt):
        msg = can.Message(arbitration_id=self.ID, data=gripperData[opt], is_extended_id=False)
        return msg
    
    def default(self):
        msg = can.Message(arbitration_id=self.ID, data=initData['gripper'], is_extended_id=False)
        return msg
    
    def status(self):
        return 0
    

class Switch():

    def __init__(self, ID):
        self.ID = ID
        self.bitData = np.zeros(8) 
        self.bitRatio = np.array([1, 2, 4, 8, 16, 32, 64, 128])
        self.data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    def bit2num(self):
        return np.sum(self.bitData * self.bitRatio)
    
    def operation(self, bit, flag):
        self.bitData[bit] = 1 if flag == True else 0
        byte0 = self.bit2num().astype(np.uint8)
        self.data[0] = byte0
        print(self.data)
        msg = can.Message(arbitration_id=self.ID, data=self.data, is_extended_id=False)
        return msg
    
    def default(self):
        msg = can.Message(arbitration_id=self.ID, data=initData['switch'], is_extended_id=False)
        return msg