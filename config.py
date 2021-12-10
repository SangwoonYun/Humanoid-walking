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

TORQUE_ENABLE           = 1     # Value for enabling the torque
TORQUE_DISABLE          = 0     # Value for disabling the torque
DXL_MIN_POSITION_VALUE  = 100   # Dynamixel will rotate between this value
DXL_MAX_POSITION_VALUE  = 4000  # and this value 
