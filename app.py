#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# app.py
#
#  Created on: 2021. 12. 18.
#      Author: Sangwoon Yun
#
import os
import copy
import time
from config import *
from kinematics import Leg
from dynamixel import Dynamixel
from zmp import ZMP

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class app(Dynamixel, ZMP):
    def __init__(self):
        super().__init__()
        ZMP().__init__()
        self.Leg_L = Leg()
        self.Leg_R = Leg()
        
    def forward_left1(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0]-STEP, DEFPOS[1]+BALEN*0.7, DEFPOS[2]+STEP_H)
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] = -15
        self.Leg_R.iv_kinematics(DEFPOS[0], DEFPOS[1]+BALEN*0.7, DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += 15
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_left1")        
        
    def forward_left2(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0]-STEP/2, DEFPOS[1]+BALEN, DEFPOS[2]+STEP_H)
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += -20
        self.Leg_R.iv_kinematics(DEFPOS[0], DEFPOS[1]+BALEN, DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += 20
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_left2")
        
    def forward_left3(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0], DEFPOS[1]+BALEN, DEFPOS[2]+STEP_H)
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        self.Leg_R.iv_kinematics(DEFPOS[0], DEFPOS[1]+BALEN, DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_left3")
    
    def forward_left4(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0]+STEP/2, DEFPOS[1]+BALEN, DEFPOS[2]+STEP_H)
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += 10
        self.Leg_R.iv_kinematics(DEFPOS[0], DEFPOS[1]+BALEN, DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += -10
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_left4")
        
    def forward_left5(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0]+STEP, DEFPOS[1]+BALEN, DEFPOS[2]+STEP_H)
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += 20
        self.Leg_R.iv_kinematics(DEFPOS[0], DEFPOS[1]+BALEN, DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += -20
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_left5")
    
    def pos_left1(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0]+STEP, DEFPOS[1]-BALEN/2, DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += 30
        self.Leg_R.iv_kinematics(DEFPOS[0], DEFPOS[1]-BALEN/2, DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += -30
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("pos_left1")
    
    def pos_left2(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0]+STEP/2, DEFPOS[1], DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += 30
        self.Leg_R.iv_kinematics(DEFPOS[0]-STEP/2, DEFPOS[1], DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += -30
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("pos_left2")
    
    def pos_left3(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0], DEFPOS[1]-BALEN/2, DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += 20
        self.Leg_R.iv_kinematics(DEFPOS[0]-STEP, DEFPOS[1]-BALEN/2, DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += -20
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("pos_left3")
    
    def forward_right1(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0], DEFPOS[1]-BALEN*0.7, DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += 20
        self.Leg_R.iv_kinematics(DEFPOS[0]-STEP, DEFPOS[1]-BALEN*0.7, DEFPOS[2]+STEP_H)
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += -20
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_right1")
    
    def forward_right2(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0], DEFPOS[1]-BALEN, DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += 10
        self.Leg_R.iv_kinematics(DEFPOS[0]-STEP/2, DEFPOS[1]-BALEN, DEFPOS[2]+STEP_H)
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += -10
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_right2")
    
    def forward_right3(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0], DEFPOS[1]-BALEN, DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        self.Leg_R.iv_kinematics(DEFPOS[0], DEFPOS[1]-BALEN, DEFPOS[2]+STEP_H)
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_right3")
    
    def forward_right4(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0], DEFPOS[1]-BALEN, DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += -10
        self.Leg_R.iv_kinematics(DEFPOS[0]+STEP/2, DEFPOS[1]-BALEN, DEFPOS[2]+STEP_H)
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += 10
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_right4")
        
    def forward_right5(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0], DEFPOS[1]-BALEN, DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += -20
        self.Leg_R.iv_kinematics(DEFPOS[0]+STEP, DEFPOS[1]-BALEN, DEFPOS[2]+STEP_H)
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += 20
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("fw_right5")
    
    def pos_right1(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0], DEFPOS[1]-BALEN/2, DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += -20
        self.Leg_R.iv_kinematics(DEFPOS[0]+STEP, DEFPOS[1]-BALEN/2, DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += 20
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("pos_right1")
        
    def pos_right2(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0]-STEP/2, DEFPOS[1], DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]
        thetasL[0] += -20
        self.Leg_R.iv_kinematics(DEFPOS[0]+STEP/2, DEFPOS[1], DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += 20
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("pos_right2")
        
    def pos_right3(self):
        pos_list = []
        self.Leg_L.iv_kinematics(DEFPOS[0]-STEP, DEFPOS[1]+BALEN/2, DEFPOS[2])
        thetasL = self.Leg_L.get_thetas();
        thetasL[-1],thetasL[-2] = thetasL[-2],thetasL[-1]  
        thetasL[0] += -20
        self.Leg_R.iv_kinematics(DEFPOS[0], DEFPOS[1]+BALEN/2, DEFPOS[2])
        thetasR = self.Leg_R.get_thetas();
        thetasR[-1],thetasR[-2] = thetasR[-2],thetasR[-1]
        thetasR[0] += 20
        for idx, val in enumerate(DEFANGLE):
            pos_list.append(self.get_angle_value(thetasR[idx], idx*2))
            pos_list.append(self.get_angle_value(thetasL[idx], idx*2+1))
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        print("pos_right3")
        
    def pos_default(self):
        pos_list = [358, 666, 512, 512, 410, 614, 307, 717, 512, 512, 614, 410]
        speeds = self.get_speed(pos_list)
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(pos_list[idx])
            param_goal_speed    = self.to_byte_array(speeds[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_add_param(self.groupSyncWriteSpeed,    DXL_ID, param_goal_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        self.sync_write_param(self.groupSyncWritePosition)
        
    def get_angle_value(self, ang, idx):
        temp = ang * WEIGHT[idx] + BIAS[idx]
        value = self.angle2value(temp)
        return int(value)
    
    def mapping(self, x, in_min, in_max, out_min, out_max):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    
    def get_speed(self, thetas):
        max_theta = max(thetas)
        min_theta = min(thetas)
        result = []
        for val in thetas:
            result.append(self.mapping(val, min_theta, max_theta, 120, 120))
        return result
    
if __name__ == "__main__":
    human = app()
    
    index = 0
    try:
        '''speed = 40
        for DXL in human.DXL_ID_LIST:
            if DXL in (40, 41, 44, 45, 46, 47):
                rspeed = speed * 2
            else:
                rspeed = speed
            param_moving_speed = human.to_byte_array(rspeed)
            human.sync_add_param(human.groupSyncWriteSpeed, DXL, param_moving_speed)
        human.sync_write_param(human.groupSyncWriteSpeed)'''
        human.pos_default()
        time.sleep(2)
        while 1:
            '''print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break'''

            if (index % 16) == 0:
                human.pos_left3()
            elif (index % 16) == 1:
                human.forward_right1()
            elif (index % 16) == 2:
                human.forward_right2()
            elif (index % 16) == 3:
                human.forward_right3()
            elif (index % 16) == 4:
                human.forward_right4()
            elif (index % 16) == 5:
                human.forward_right5()
            elif (index % 16) == 6:
                human.pos_right1()
            elif (index % 16) == 7:
                human.pos_right2()
            elif (index % 16) == 8:
                human.pos_right3()
            elif (index % 16) == 9:
                human.forward_left1()
            elif (index % 16) == 10:
                human.forward_left2()
            elif (index % 16) == 11:
                human.forward_left3()
            elif (index % 16) == 12:
                human.forward_left4()
            elif (index % 16) == 13:
                human.forward_left5()
            elif (index % 16) == 14:
                human.pos_left1()
            elif (index % 16) == 15:
                human.pos_left2()

            #while human.are_moving():
            #    pass
            time.sleep(0.15)
            index += 1
        human.pos_default()
        time.sleep(1)
        human.terminate()
            
    except Exception as e:
        print(e)
        human.pos_default()
        time.sleep(1)
        human.terminate()
    except KeyboardInterrupt:
        human.pos_default()
        time.sleep(1)
        human.terminate()
        