#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# zmp.py
#
#  Created on: 2021. 12. 10.
#      Author: Sangwoon Yun
#

import numpy as np
import math

from DHmatrix import DH

if __name__ == "__main__":
    np.set_printoptions(precision=1, suppress=True)
	
def x_zmp():
	param1, param2, param3 = 0
	for i in range(n):
		param1 += m[i]*(pos_z[i] + gravity)*center[0]
		param2 += m[i]*center[0]*pos_z[i]
		param3 += m[i]*(pos_z[i] + gravity)

	result = (param1-param2)/param3
	return result

def y_zmp():
	param1, param2, param3 = 0
	for i in range(n):
		param1 += m[i]*(pos_z[i] + gravity)*center[1]
		param2 += m[i]*center[1]*pos_z[i]
		param3 += m[i]*(pos_z[i] + gravity)

	result = (param1-param2)/param3

def detect_point(polygon, zmp):
	point = polygon
	point.append(polygon[0])
	lines = len(polygon)

	zx = zmp[0]
	zy = zmp[1]
	count = 0

	for i in range(lines):
		x1 = point[i][0]
		y1 = point[i][1]
		x2 = point[i+1][0]
		y2 = point[i+1][1]

		# check zmp is in polygon or not
		## filter point for checking 
		if (x1 < zx) and (x2 < zx):
			continue
		if (y1 < zy) and (y2 < zy):
			continue
		
		x = (x1-x2)*(zy-y1)/(y1-y2) + x1
		if (x1-x)*(x2-x) < 0:
			count += 1
	
	if count % 2 == 0:
		return False
	else
		return True


