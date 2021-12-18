#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# app.py
#
#  Created on: 2021. 12. 18.
#      Author: Sangwoon Yun
#
import os
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
        
    def forward_left(self):
        dxl_goal_position = [358, 666, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512]
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(dxl_goal_position[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
        self.sync_write_param(self.groupSyncWritePosition)
    
    def forward_right(self):
        dxl_goal_position = [358, 666, 512, 512, 308, 716, 102, 922, 512, 512, 716, 308]
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(dxl_goal_position[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
        self.sync_write_param(self.groupSyncWritePosition)
    
    def pos_left(self):
        dxl_goal_position = [358, 666, 512, 512, 410, 614, 307, 717, 512, 512, 614, 410]
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(dxl_goal_position[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
        self.sync_write_param(self.groupSyncWritePosition)
    
    def pos_right(self):
        dxl_goal_position = [358, 666, 512, 512, 410, 614, 307, 717, 512, 512, 614, 410]
        for idx, DXL_ID in enumerate(self.DXL_ID_LIST):
            param_goal_position = self.to_byte_array(dxl_goal_position[idx])
            self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
        self.sync_write_param(self.groupSyncWritePosition)
        
    
if __name__ == "__main__":
    human = app()
    
    index = 0
    try:
        speed = 150
        for DXL in human.DXL_ID_LIST:
            if DXL in (44, 45):
                rspeed = speed * 2
            else:
                rspeed = speed
            param_moving_speed = human.to_byte_array(rspeed)
            human.sync_add_param(human.groupSyncWriteSpeed, DXL, param_moving_speed)
        human.sync_write_param(human.groupSyncWriteSpeed)
        
        while 1:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break

            if (index % 4) == 0:
                human.pos_left()
            elif (index % 4) == 1:
                human.forward_right()
            elif (index % 4) == 2:
                human.pos_right()
            elif (index % 4) == 3:
                human.forward_left()

            while human.are_moving():
                pass
            index += 1
        human.terminate()
            
    except Exception as e:
        print(e)
        human.terminate()
        
