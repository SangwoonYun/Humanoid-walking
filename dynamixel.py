#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# dynamixel.py
#
#  Created on: 2021. 12. 12.
#      Author: Sangwoon Yun
#

import os
from config import *

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

from dynamixel_sdk import *
import time

class Dynamixel():
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWritePosition = GroupSyncWrite(self.portHandler,
                                                     self.packetHandler,
                                                     ADDR_AX_GOAL_POSITION,
                                                     LEN_AX_GOAL_POSITION)
        self.groupSyncWriteSpeed    = GroupSyncWrite(self.portHandler,
                                                     self.packetHandler,
                                                     ADDR_AX_MOVING_SPEED,
                                                     LEN_AX_MOVING_SPEED)
        
        self.groupSyncReadMoving    = GroupSyncRead(self.portHandler,
                                                    self.packetHandler,
                                                    ADDR_AX_IS_MOVING,
                                                    LEN_AX_IS_MOVING)
        self.open_port()
        self.set_port_baudrate()
        self.DXL_ID_LIST = DXL_ID_L + DXL_ID_R
        self.DXL_ID_LIST.sort()
        for DXL_ID in self.DXL_ID_LIST:
            self.enable_dynamixel_torque(DXL_ID)

    def terminate(self):
        for DXL_ID in DXL_ID_L+DXL_ID_R:
            self.disable_dynamixel_torque(DXL_ID)
        self.close_port()

    def open_port(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def close_port(self):
        self.portHandler.closePort()

    def set_port_baudrate(self):
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def enable_dynamixel_torque(self, DXL_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,
                                                                       DXL_ID, 
                                                                       ADDR_AX_TORQUE_ENABLE, 
                                                                       TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] has been successfully connected" % (DXL_ID))

    def disable_dynamixel_torque(self, DXL_ID):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,
                                                                       DXL_ID,
                                                                       ADDR_AX_TORQUE_ENABLE,
                                                                       TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] has been successfully disconnected" % (DXL_ID))

    def write_position(self, DXL_ID, dxl_goal_position):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler,
                                                                       DXL_ID,
                                                                       ADDR_AX_GOAL_POSITION,
                                                                       dxl_goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def read_position(self, DXL_ID):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,
                                                                                            DXL_ID,
                                                                                            ADDR_AX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return -1
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return -1
        else:
            return dxl_present_position

    def write_speed(self, DXL_ID, dxl_moving_speed):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler,
                                                                       DXL_ID,
                                                                       ADDR_AX_MOVING_SPEED,
                                                                       dxl_moving_speed)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def sync_add_param(self, groupSync, DXL_ID, param):
        dxl_addparam_result = groupSync.addParam(DXL_ID, param)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSync addparam failed" % DXL_ID)
            quit()
            
    def sync_add_read(self, groupSync, DXL_ID):
        dxl_addparam_result = groupSync.addParam(DXL_ID)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSync addparam failed" % DXL_ID)
            quit()

    def sync_write_param(self, groupSync):
        dxl_comm_result = groupSync.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        groupSync.clearParam()
        
    def sync_read_param(self, groupSync):
        dxl_comm_result, result = groupSync.rxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        groupSync.clearParam()
        return result

    def to_byte_array(self, value):
        param_array = [DXL_LOBYTE(value),
                       DXL_HIBYTE(value)]
        return param_array

    def is_done(self, dxl_goal_position, dxl_present_position):
        if abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            return False
        else:
            return True
        
    def are_moving(self):
        for DXL_ID in self.DXL_ID_LIST:
            dxl_is_moving, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler,
                                                                                         DXL_ID,
                                                                                         ADDR_AX_IS_MOVING)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                return -1
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                return -1
            elif dxl_is_moving == 1:
                return 1
        return 0
        '''for DXL_ID in self.DXL_ID_LIST:
            self.sync_add_read(self.groupSyncReadMoving, DXL_ID)
        dxl_is_moving = self.sync_read_param(self.groupSyncReadMoving)
        print(dxl_is_moving)
        return 0'''

    def angle2real(self, angle):
        rangle = angle + 150
        return rangle

    def angle2value(self, angle):
        value = angle * VALUEPANG
        return value

    def run(self):  # test code
        index = 0
        DXL_LIST = DXL_ID_L + DXL_ID_R
        DXL_LIST.sort()
        speed = 150
        for DXL in DXL_LIST:
            if DXL in (44, 45):
                rspeed = speed * 2
            else:
                rspeed = speed
            param_moving_speed = self.to_byte_array(rspeed)
            self.sync_add_param(self.groupSyncWriteSpeed, DXL, param_moving_speed)
        self.sync_write_param(self.groupSyncWriteSpeed)
        dxl_goal_position = [[358, 666, 512, 512, 512, 512, 512, 512, 512, 512, 512, 512],  # 0
                             [358, 666, 512, 512, 461, 563, 410, 615, 512, 512, 563, 461],  # 15
                             [358, 666, 512, 512, 410, 614, 307, 717, 512, 512, 614, 410],  # 30
                             [358, 666, 512, 512, 359, 665, 205, 819, 512, 512, 665, 359],  # 45
                             [358, 666, 512, 512, 308, 716, 102, 922, 512, 512, 716, 308],  # 60
                             [358, 666, 512, 512, 359, 665, 205, 819, 512, 512, 665, 359],  # 45
                             [358, 666, 512, 512, 410, 614, 307, 717, 512, 512, 614, 410],  # 30
                             [358, 666, 512, 512, 461, 563, 410, 615, 512, 512, 563, 461]]  # 15

        while 1:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break

            for idx, DXL_ID in enumerate(DXL_LIST):
                param_goal_position = self.to_byte_array(dxl_goal_position[index][idx])
                self.sync_add_param(self.groupSyncWritePosition, DXL_ID, param_goal_position)
            self.sync_write_param(self.groupSyncWritePosition)
            if index == len(dxl_goal_position)-1:
                index = 0
            else:
                index += 1
        self.terminate()

if __name__ == '__main__':
    test = Dynamixel()
    try:
        test.run()
    except Exception as e:
        print(e)
        test.terminate()
