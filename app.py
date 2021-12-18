#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# app.py
#
#  Created on: 2021. 12. 18.
#      Author: Sangwoon Yun
#

from kinemaics import Leg
from dynamixel import Dynamixel

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

class app(ZMP):
    def __init__(self):
        Leg_L = Leg()
        Leg_R = Leg()
        DLX   = Dynamixel()
        
    def forward_left(self):
        pass
    
    def forward_right(self):
        pass
    
    def pos_left(self):
        pass
    
    def pos_right(self):
        pass
        
    
if __name__ == "__main__":
    np.set_printoptions(precision=1, suppress=True)
    human = app()
    
    while 1:
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break
