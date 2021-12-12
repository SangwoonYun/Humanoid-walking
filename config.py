#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# config.py
#
#  Created on: 2021. 12. 11.
#      Author: Sangwoon Yun
#

# DH parameters
L3 = 74
L4 = 77
L6 = 33


# Dynamixel SDK
ADDR_AX_TORQUE_ENABLE       = 24
ADDR_AX_GOAL_POSITION       = 30
ADDR_AX_PRESENT_POSITION    = 36
ADDR_AX_MOVING_SPEED        = 32
LEN_AX_GOAL_POSITION        = 2
LEN_AX_PRESENT_POSITION     = 2
LEN_AX_MOVING_SPEED         = 2

PROTOCOL_VERSION            = 1.0
BAUDRATE                    = 1000000
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 20

DXL_ID_L   = [id for id in range(39, 50, 2)]
DXL_ID_R   = [id for id in range(38, 49, 2)]

ANGLELIMIT = 300
VALUELIMIT = 1023
ANGLEPVAL  = ANGLELIMIT/(VALUELIMIT+1)
VALUEPANG  = (VALUELIMIT+1)/ANGLELIMIT
DEFANGLEL  = (195, 150, 150, 150, 150, 150)
DEFANGLER  = (105, 150, 150, 150, 150, 150)


