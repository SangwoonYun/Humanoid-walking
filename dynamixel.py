#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# dynamixel.py
#
#  Created on: 2021. 12. 11.
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

DXL_ID = DXL_ID_L[0]

class Dynamixel():
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.open_port()
        self.set_port_baudrate()
        self.enable_dynamixel_torque()

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

    def enable_dynamixel_torque(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,
                                                                       DXL_ID, 
                                                                       ADDR_MX_TORQUE_ENABLE, 
                                                                       TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")

    def disable_dynamixel_torque(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler,
                                                                       DXL_ID,
                                                                       ADDR_MX_TORQUE_ENABLE,
                                                                       TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def run(self):
        index = 0
        while 1:
            print("Press any key to continue! (or press ESC to quit!)")
            if getch() == chr(0x1b):
                break

            dxl_goal_position = [276, 358, 440]
            self.write_position(dxl_goal_position[index])
            
            while 1:
                dxl_present_position = self.read_position()
                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))
                if self.is_done(dxl_goal_position[index], dxl_present_position):
                    if index == 2:
                        index = 0
                    else:
                        index += 1
                    break
        self.disable_dynamixel_torque()
        self.close_port()

    def write_position(self, dxl_goal_position):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler,
                                                                       DXL_ID,
                                                                       ADDR_MX_GOAL_POSITION,
                                                                       dxl_goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    def read_position(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,
                                                                                            DXL_ID,
                                                                                            ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return -1
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return -1
        else:
            return dxl_present_position

    def is_done(self, dxl_goal_position, dxl_present_position):
        if abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            return False
        else:
            return True

if __name__ == '__main__':
    test = Dynamixel()
    try:
        test.run()
    except KeyboardInterrupt:
        test.disable_dynamixel_torque()
        test.close_port()
