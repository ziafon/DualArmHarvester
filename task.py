#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
from config import heightThreshold

# 这里需要重新测量
Hstart = 0.95
Hlen = 2.150 - 0.95
Wstart = 0.150
Wlen = 1.781 - 0.150

horizontal_protection = 0.2 # 水平方向双臂碰撞保护距离
vertical_protection = 0.4 # 垂直方向双臂碰撞保护距离


def simple_plan(objects, Targets):
    objects = np.array(objects).reshape(-1, 3)
    idx = np.where(objects[:, 2] < heightThreshold)
    Targets['lower'] = objects[idx]
    Targets['upper'] = np.delete(objects, idx, axis=0)
    return Targets







 






    










