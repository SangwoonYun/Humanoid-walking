#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# zmp.py
#
#  Created on: 2021. 12. 18.
#      Author: Sangwoon Yun
#

import numpy as np
import math

from config import *
from DHmatrix import DH
  
class ZMP():
    def __init__(self):
        self.acc_x = 0 # Assume Constant Velocity
        self.acc_y = 0
        self.g = 9.8
        self.T_F = T_F
        self.T_R = T_R
        self.T_S = T_S
        
    def get_x_zmp(self, z_com, x_com):
        x_zmp = x_com - (z_com / self.g) * self.acc_x
        return x_zmp

    def get_y_zmp(self, z_com, y_com):
        y_zmp = y_com - (z_com / self.g) * self.acc_y
        return y_zmp
        
    def get_polygon(self, x, y):
        x_min = x - self.T_R
        x_max = x + self.T_F
        y_min = y - self.T_S
        y_max = y + self.T_S
        return ((x_max, y_max), (x_min, y_min))
    
if __name__ == "__main__":
    np.set_printoptions(precision=1, suppress=True)