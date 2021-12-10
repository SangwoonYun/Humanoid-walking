#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# kinematics.py
#
#  Created on: 2021. 12. 09.
#      Author: Sangwoon Yun
#

import numpy as np
import math

from config import *
from DHmatrix import DH

class LEG(): 
    def __init__(self):
        self.H = []
        self.set_origin()
        self.H.append(DH( 0, 90,  0, 0)) # hip_yaw
        self.H.append(DH( 0,-90,  0, 0)) # hip_roll
        self.H.append(DH( 0,  0, L3, 0)) # hip_pitch
        self.H.append(DH( 0,  0, L4, 0)) # knee_pitch
        self.H.append(DH( 0, 90,  0, 0)) # ankle_pitch
        self.H.append(DH( 0,  0, L6, 0)) # ankle_roll
        self.set_skiplate()
        self.fw_kinematics()

    def set_origin(self):
        DH_ORG = DH()
        DH_ORG.H = np.array([[ 0,-1, 0, 0],
                             [ 0, 0, 1, 0], 
                             [-1, 0, 0, 0],
                             [ 0, 0, 0, 1]])
        self.H.append(DH_ORG)

    def set_skiplate(self):
        DH_SKI = DH()
        DH_SKI.H = np.array([[ 0, 0,-1, 0],
                             [ 0, 1, 0, 0], 
                             [ 1, 0, 0, 0],
                             [ 0, 0, 0, 1]])
        self.H.append(DH_SKI)

    def fw_kinematics(self):    # get_foot_position_T
        Tg = []
        Tg.append(self.H[0])
        for idx, H in enumerate(self.H[1:]):
            Tg.append(Tg[idx] * H)
        self.Tg = Tg

    def get_thetas(self):
        thetas = []
        for H in self.H[1:-1]:
            thetas.append(H.get_theta())
        return thetas

    def set_thetas(self, thetas):
        for idx, H in enumerate(self.H[1:-1]):
            H.set_theta(thetas[idx])

    def get_T(self):
        return self.Tg[-1]

    def get_foot(self): # return (x, y, z)
        return (self.Tg[-1].H[0,3], self.Tg[-1].H[1,3], self.Tg[-1].H[2,3])

    def get_ankle(self):
        return (self.Tg[4].H[0,3],  self.Tg[4].H[1,3],  self.Tg[4].H[2,3])

    def get_knee(self):
        return (self.Tg[3].H[0,3],  self.Tg[3].H[1,3],  self.Tg[3].H[2,3])
    
    def get_theta1(self):
        r12 = self.get_T().H[0,1]
        r13 = self.get_T().H[0,2]
        r22 = self.get_T().H[1,1]
        r23 = self.get_T().H[1,2]
        theta2 = self.H[2].get_theta(rad=True)
        theta6 = self.H[6].get_theta(rad=True)
        cos_theta2 = math.cos(theta2)
        sin_theta6 = math.sin(theta6)
        cos_theta6 = math.cos(theta6)
        templn = r12 * cos_theta6 - r13 * sin_theta6
        templd = -cos_theta2
        templ = templn / templd
        temprn = r22 * cos_theta6 - r23 * sin_theta6
        temprd = cos_theta2
        tempr = temprn / temprd
        theta1 = math.atan2(templ, tempr)
        return theta1

    def get_theta2(self):
        r32 = self.get_T().H[2,1]
        r33 = self.get_T().H[2,2]
        theta6 = self.H[6]._theta
        templ = r32 * math.cos(theta6)
        tempr = r33 * math.sin(theta6)
        theta2 = math.asin(templ - tempr)
        return theta2

    def get_theta3(self, px, py, pz):
        theta1 = self.H[1].get_theta(rad=True)
        theta2 = self.H[2].get_theta(rad=True)
        theta4 = self.H[4].get_theta(rad=True)
        sin_theta1 = math.sin(theta1)
        cos_theta1 = math.cos(theta1)
        sin_theta2 = math.sin(theta2)
        cos_theta2 = math.cos(theta2)
        sin_theta4 = math.sin(theta4)
        cos_theta4 = math.cos(theta4)
        l3  = self.H[3].l
        l4 = self.H[4].l
        l6 = self.H[6].l
        r13 = self.get_T().H[0,2]
        r23 = self.get_T().H[1,2]
        r33 = self.get_T().H[2,2]
        tempa1 = -px * sin_theta1 * sin_theta2
        tempa2 =  py * cos_theta1 * sin_theta2
        tempa3 = -pz * cos_theta2
        tempa41 = r13 * sin_theta1 * sin_theta2
        tempa42 = r23 * cos_theta1 * sin_theta2
        tempa43 = r33 * cos_theta2
        alpha = tempa1 + tempa2 + tempa3 - l6 * (tempa41 - tempa42 + tempa43)
        beta  = -px * cos_theta1 - py * sin_theta1 - l6 * (r13 * cos_theta1 + r23 * sin_theta1)
        templ = -l4 * sin_theta4 * alpha + (l3 + l4 * cos_theta4) * beta
        tempr =  l4 * sin_theta4 * beta  + (l3 + l4 * cos_theta4) * alpha
        theta3 = math.atan2(templ, tempr)
        return theta3

    def get_l05(self, px, py, pz):
        P = self.get_T().H
        l6 = self.H[6].l
        tempx = px + l6 * P[0,2]
        tempy = py + l6 * P[1,2]
        tempz = pz + l6 * P[2,2]
        l05 = (tempx**2 + tempy**2 + tempz**2)**0.5
        return l05

    def get_theta4(self, px, py, pz):
        l3  = self.H[3].l
        l4  = self.H[4].l
        l05 = self.get_l05(px, py, pz)
        tempn = l05**2 - l3**2 - l4**2
        tempd = 2 * l3 * l4
        try:
            theta4 = math.acos(tempn / tempd)
        except ValueError:
            print('theta4 Error :', tempn, tempd)
            theta4 = 0
        return theta4

    def get_theta5(self):
        r11 = self.get_T().H[0,0]
        r21 = self.get_T().H[1,0]
        r31 = self.get_T().H[2,0]
        theta1 = self.H[1].get_theta(rad=True)
        theta2 = self.H[2].get_theta(rad=True)
        sin_theta1 = math.sin(theta1)
        cos_theta1 = math.cos(theta1)
        sin_theta2 = math.sin(theta2)
        cos_theta2 = math.cos(theta2)
        templ = -r11 * sin_theta1 * sin_theta2 + r21 * cos_theta1 * sin_theta2 - r31 * cos_theta2
        tempr = r11 * cos_theta1 + r21 * sin_theta1
        theta_pitch = math.atan2(templ, tempr)
        theta5 = theta_pitch - self.H[3]._theta - self.H[4]._theta
        return theta5

    def get_theta6(self, px, py, pz):
        Pi = np.linalg.inv(self.get_T().H)
        Pi24 = Pi[1,3]
        Pi34 = Pi[2,3]
        l6 = self.H[6].l
        l06yz = math.sqrt(Pi24**2 + Pi34**2)
        l05yz = math.sqrt(Pi24**2 + (Pi34 - l6)**2)
        templ = np.sign(Pi24)
        temprn = l06yz**2 - l05yz**2 - l6**2
        temprd = 2 * l05yz * l6
        try:
            tempr = math.acos(temprn / temprd)
        except ValueError:
            print('theta6 Error :', temprn, temprd)
            tempr = 0
        theta6 = templ * tempr
        return theta6

    def iv_kinematics(self, px, py, pz):
        self.Tg[-1].set_euler()
        self.Tg[-1].H[0,3] = px
        self.Tg[-1].H[1,3] = py
        self.Tg[-1].H[2,3] = pz
        theta4 = self.get_theta4(px, py, pz)
        self.H[4].set_theta(theta4, rad=True)
        theta6 = self.get_theta6(px, py, pz)
        self.H[6].set_theta(theta6, rad=True)
        theta2 = self.get_theta2()
        self.H[2].set_theta(theta2, rad=True)
        theta1 = self.get_theta1()
        self.H[1].set_theta(theta1, rad=True)
        theta3 = self.get_theta3(px, py, pz)
        self.H[3].set_theta(theta3, rad=True)
        theta5 = self.get_theta5()
        self.H[5].set_theta(theta5, rad=True)

if __name__ == "__main__":
    np.set_printoptions(precision=1, suppress=True)
    leg_left = LEG()
    
    thetas = [-45, 30, -60, 90, -30, -30]
    print(["{0:0.2f}".format(i) for i in thetas])
    leg_left.set_thetas(thetas)
    leg_left.fw_kinematics()
    print(leg_left.get_knee())
    print(leg_left.get_ankle())
    print(leg_left.get_foot())
    #print(leg_left.get_T())'''
    
    foot = leg_left.get_foot()
    leg_left.iv_kinematics(foot[0], foot[1], foot[2])
    thetas = leg_left.get_thetas();
    print(["{0:0.2f}".format(i) for i in thetas])
    leg_left.set_thetas(thetas)
    leg_left.fw_kinematics()
    print(leg_left.get_knee())
    print(leg_left.get_ankle())
    print(leg_left.get_foot())
    #print(leg_left.get_T())'''
