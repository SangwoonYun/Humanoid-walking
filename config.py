#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# config.py
#
#  Created on: 2021. 12. 10.
#      Author: Sangwoon Yun
#

# DH parameters
L3 = 74
L4 = 77
L6 = 33

# Dynamixel SDK
DXL_ID_L   = [id for id in range(38, 49, 2)]
DXL_ID_R   = [id for id in range(39, 50, 2)]
BAUDRATE   = 1000000
DEVICENAME = "/dev/ttyUSB0".encode('utf-8')
ANGLELIMIT = 300
VALUELIMIT = 1023
ANGLEPVAL  = ANGLELIMIT/(VALUELIMIT+1)
VALUEPANG  = (VALUELIMIT+1)/ANGLELIMIT
DEFANGLEL  = (195, 150, 150, 150, 150, 150)
DEFANGLER  = (105, 150, 150, 150, 150, 150)

TORQ_ENABLE  = 1    # Value for enabling the torque
TORQ_DISABLE = 0    # Value for disabling the torque
DXL_POS_LMTL = [(358,666), (), (),
                (), (), ()]   # Dynamixel will rotate limit
DXL_POS_LMTR = []
