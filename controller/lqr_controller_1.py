#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version by Ken Lauer / sirkne at gmail dot com
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

"""
lqr_controller.py
LQR Controller

Author: Sam Staszak
"""
import math
import numpy as np
import time

class LQRController:
    def __init__(self, dynamics, cost):
		self.d = dynamics
		self.c = cost	
		self.eps = 10.0**-6


    def computeA(self, x_t, u_t, t):
		d_f_x = self.getFiniteDiff1(self.d, x_t, u_t, True)
		d_f_u = self.getFiniteDiff1(self.d, x_t, u_t, False)
		a_12 = self.d.solve(x_t, u_t) - self.xTars[t+1] +\
			d_f_x*(self.xTars[t] - x_t) + d_f_u*(self.uTars[t] - u_t)
		A_t = np.matrix([[d_f_x, a_12],
						[0, 1]])
		return A_t

    def computeB(self, x_t, u_t, t):
		b_11 = self.getFiniteDiff1(self.d, x_t, u_t, False)
		B_t = np.matrix([[b_11],
				[0]])

	def genericQR(self):
		

    def computeQR(self, x_t, u_t, t):
		d_g_x = self.getFiniteDiff1(self.c, x_t, u_t, True)
		d_g_u = self.getFiniteDiff1(self.c, x_t, u_t, False)
		d_g_xx = self.getFiniteDiff2(self.c, x_t, u_t, True)
		d_g_xu = self.getFiniteDiff2XY(self.c, x_t, u_t)
		d_g_uu = self.getFiniteDiff2(self.c, x_t, u_t, False)

		uBar = (self.uTars[t] - u_t)
		xBar = (self.xTars[t] - x_t)

		q_11 = (.5*d_g_xx)
		q_12 = (d_g_xu * uBar)
		q_21 = d_g_x
		q_22 = self.c.solve(x_t, u_t) - self.c.solve(self.xTars[t], self.uTars[t]) +\
		d_g_x*xBar + d_g_u*uBar + (.5*xBar*d_g_xx*xBar) + (.5*uBar*d_g_uu*uBar)

		Q = np.matrix([[q_11, q_12],
				[q_21, q_22]])

		R = np.matrix([[(.5*d_g_uu)]])

		return [Q, R]

    def getDelta(self, var):
		return var * self.eps

    def getFiniteDiff1(self, f, x, y, firstTerm):
		if firstTerm:
		    d_x = self.getDelta(x)
	 	   return (f.solve(x + d_x, y) -\
					f.solve(x - d_x, y)) / (2*d_x)
		else:
	  		d_y = self.getDelta(y)
	    	return (f.solve(x, y + d_y) -\
	        		f.solve(x, y - d_y)) / (2*d_y)

    def getFiniteDiff2(self, f, x, y, firstTerm):
		if firstTerm:
		    d_x = self.getDelta(x)
		    return (f.solve(x + d_x, y) -\
					2*f.solve(x, y) +\
		        	f.solve(x - d_x, y)) / (d_x**2)
		else:
		    d_y = self.getDelta(y)
		    return (f.solve(x, y + d_y) -\
					2*f.solve(x, y) +\
		        	f.solve(x, y - d_y)) / (d_y**2)

    def getFiniteDiff2XY(self, f, x, y):
		d_x = self.getDelta(x)
		d_y = self.getDelta(y)
		return (f.solve(x + d_x, y + d_y) -\
				f.solve(x + d_x, y - d_y) -\
		        f.solve(x - d_x, y + d_y) +\
				f.solve(x - d_x, y - d_y)) / (4*d_x*d_y)
	    


    def setTargetControls(self, x_tars, u_tars):
		self.xTars = x_tars
		self.uTars = u_tars

    def setInitialState(self, x_1):
		self.x1 = x_1


class Cost:
    def __init__(self, func):
		self.func = func
	

    def solve(self, x_t, u_t):
	


class Dynamics:
	def __init__(self):
		

def main():



if __name__ == '__main__':
    main()
