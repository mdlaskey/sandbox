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
from sympy import *
import time

class LQRController:
    def __init__(self, dynamics, cost = None):
	self.d = dynamics
	self.c = cost	
	self.eps = 10.0**-6

    def computeA(self, x_t, u_t, t):
	d_f_x = self.getFiniteDiff1(self.d, x_t, u_t, True)
        
        print("d_f_x: " + str(d_f_x))

	a_12 = self.d.solve(x_t, u_t) - self.xCurr[t+1]
	A_t = np.matrix([[d_f_x, a_12],
                         [0, 1]])
	return A_t

    def computeB(self, x_t, u_t, t):
	b_11 = self.getFiniteDiff1(self.d, x_t, u_t, False)
	B_t = np.matrix([[b_11],
                         [0]])
        return B_t

    def genericQR(self):
        pass		

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
 	    top = (f.solve(x + d_x, y) -\
                       f.solve(x - d_x, y))
            bottom = 2 * d_x
            
	else:
            d_y = self.getDelta(y)
	    top = (f.solve(x, y + d_y) -\
                       f.solve(x, y - d_y))
            bottom = 2 * d_y

        for i in range(top.shape[1] - 1):
            top[i, 0] = top[i, 0] / bottom[i, 0]
        
        return top

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
	    


    def setCurrentPolicy(self, x_curr, u_curr):
	self.xCurr = x_curr
	self.uCurr = u_curr

    def setInitialState(self, x_1):
	self.x1 = x_1

    def doMulti(self, M, N):
        r = M.shape[0]
        c = N.shape[1]
        L = np.zeros(shape=(r, c), dtype=object)
        for i in range(r):
            for j in range(c):
                for k in range(N.shape[0]):
                    L[i, j] += np.multiply(M[i, k], N[k, j])

        print ("L: " + str(L))

        return L
            


class Cost:
    def __init__(self, func):
	self.func = func
	
    def solve(self, x_t, u_t):
	pass


class Dynamics:
    def __init__(self, x_init, u_init):
	#t = Symbol('t')
	#u_s = Symbol('u_s')
	#u_p = Symbol('u_p')

        self.xs = [x_init]
        self.us = [u_init]

    def solve(self, X, U):
	dx = U[0,0] * math.cos(X[2,0])
	dy = U[0,0] * math.sin(X[2,0])
	dt = U[0,0] * math.tan(U[1,0])
	N = np.matrix([[dx], [dy], [dt]])
		
        return N

    def setInit(self, x_0, u_0):
        self.x0 = x_0
        self.u0 = u_0

    def addIteration(self, x_t, u_t):
        self.xs.append(x_t)
        self.us.append(u_t)

def main():
	x_init = np.matrix([[1], [1], [math.pi/3.0]])
	u_init = np.matrix([[3], [math.pi/6.0]])
	
	d = Dynamics(x_init, u_init)
	lqr = LQRController(d)

        xGuess = [x_init, d.solve(x_init, u_init)]
        print("Solution: " + str(d.solve(x_init, u_init)))
        uGuess = [u_init, np.matrix([[1], [math.pi/4]])]
	
        lqr.setCurrentPolicy(xGuess, uGuess)

	A = lqr.computeA(x_init, u_init, 0)
        B = lqr.computeB(x_init, u_init, 0)
        print ("A: " + str(A))
        print ("B: " + str(B))

        #D = lqr.doMulti(A, np.matrix([[x_init], [1]]))+ lqr.doMulti(B, np.matrix([[u_init]]))

        #D = A * np.matrix([[x_init], [1]]) + B * np.matrix([u_init])
        #print ("D: " + str(D))



if __name__ == '__main__':
    main()
