#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# DHmatrix.py
#
#  Created on: 2021. 12. 09.
#      Author: Sangwoon Yun
#

import numpy as np
import math

class DH():
    def __init__(self, theta=0, alpha=0, l=0, d=0, th_min=-90, th_max=90):
        self.H = np.zeros(16).reshape(4, 4)
        self.set_param(theta, alpha, l, d)
        self.set_H()
        self.th_min = th_min
        self.th_max = th_max

    def set_param(self, theta, alpha, l, d):
        self.dtheta = theta  # default_theta
        self.alpha  = alpha  # link_twist_deg
        self.l      = l      # link_lenght_m
        self.d      = d      # joint+offset_m
        self._alpha = math.radians(self.alpha) # link_twist_rad
        self.cos_alpha = math.cos(self._alpha)
        self.sin_alpha = math.sin(self._alpha)
        self.set_theta(0)

    def set_theta(self, value=0, rad=False):
        if rad:
            value = math.degrees(value)
        self.theta  = self.dtheta + value # joint_angle_deg
        self._theta = math.radians(self.theta) # joint_angle_rad
        self.cos_theta = math.cos(self._theta) 
        self.sin_theta = math.sin(self._theta)
        self.set_H()

    def get_theta(self, rad=False, dtheta=False):
        if dtheta:
            theta = self.theta
        else:
            theta = self.theta - self.dtheta
        if rad:
            return math.radians(theta)
        else:
            return theta

    def set_H(self):
        self.H[0,0] =  self.cos_theta
        self.H[0,1] = -self.cos_alpha * self.sin_theta
        self.H[0,2] =  self.sin_alpha * self.sin_theta
        self.H[0,3] =  self.cos_theta * self.l
        self.H[1,0] =  self.sin_theta
        self.H[1,1] =  self.cos_alpha * self.cos_theta
        self.H[1,2] = -self.sin_alpha * self.cos_theta
        self.H[1,3] =  self.sin_theta * self.l
        self.H[2,1] =  self.sin_alpha
        self.H[2,2] =  self.cos_alpha
        self.H[2,3] =  self.d
        self.H[3,3] =  1

    def set_euler(self, _roll=0, _pitch=0, _yaw=0):
        roll  = math.radians(_roll)
        pitch = math.radians(_pitch)
        yaw   = math.radians(_yaw)
        sin_roll  = math.sin(roll)
        cos_roll  = math.cos(roll)
        sin_pitch = math.sin(pitch)
        cos_pitch = math.cos(pitch)
        sin_yaw   = math.sin(yaw)
        cos_yaw   = math.cos(yaw)
        self.H[0,0] =  cos_yaw   * cos_pitch
        self.H[0,1] =  sin_roll  * sin_pitch * cos_yaw  + cos_roll * sin_yaw
        self.H[0,2] = -cos_yaw   * sin_pitch * cos_roll + sin_roll * sin_yaw
        self.H[1,0] = -cos_pitch * sin_yaw
        self.H[1,1] = -sin_roll  * sin_pitch * sin_yaw  + cos_roll * cos_yaw
        self.H[1,2] =  cos_roll  * sin_pitch * sin_yaw  + sin_roll * cos_yaw
        self.H[2,0] =  sin_pitch
        self.H[2,1] = -sin_roll  * cos_pitch
        self.H[2,2] =  cos_roll  * cos_pitch

    def __mul__(self, other):
        DH_DOT = DH()
        DH_DOT.H = self.H.dot(other.H)
        return DH_DOT

    def __str__(self):
        return str(self.H)

    def __repr__(self):
        return self.H
