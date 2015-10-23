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
lqr_controller_new.py
LQR Controller NEW

Author: Sam Staszak
"""
import math
import numpy as np
from sympy import *
import time

class LQRController:
    def __init__(self, dynamics, x_0, x_f, cost, u, dt, its, tol):
	self.d = dynamics
        self.x0 = x_0
        self.xf = x_f
	self.c = cost
        self.u = u
        self.dt = dt
        self.its = its
        self.tol = tol
	self.eps = 10.0**-6
        
        
    def runiLQR(self):
        i = 0
        traj = self.d.getTrajectory(self.x0, self.u)
        currCost = self.c.solve(traj, self.u, self.dt)
        oldCost = currCost
        while (i < self.its):
            itr = Iteration(traj, self.xf, self.u, self.d, self.c, self.eps)
            du = itr.solveLQR()
            self.u += du
            
            traj = self.d.getTrajectory(self.x0, self.u)
            currCost = self.cost.solve(traj, self.u, self.dt)

            if currCost < oldCost * self.tol:
                break

            oldCost = currCost

        return self.u
        


class Iteration:
    def __init__(self, x, x_f, u, d, c, eps):
        
        # current trajectory
        self.x = x
        self.u = u
        self.x_f = x_f
        self.eps = eps
        
        # dynamics and cost functions
        self.d = d
        self.c = c

    def solveLQR(self):
        # iterations
        n = self.x.shape[0]-1
        print ("shape x: " + str(self.x.shape))
        # soon to be change in u
        du = np.zeros(shape=self.u.shape, dtype=object)
        dx = 10.0**-9 * np.ones(shape=self.x.shape, dtype=object)
        print ("dx: " + str(dx))

        K = np.zeros(shape=(1, n), dtype=object)
        K_v = np.zeros(shape=(1, n), dtype=object)
        K_u = np.zeros(shape=(1, n), dtype=object)

        S = np.zeros(shape=(1, n+1), dtype=object)
        v = np.zeros(shape=(1, n+1), dtype=object)

        # need to still find Q_f this line is wrong
        [Q_f, R_f] = self.c.approximate(self.x[n], self.u[n-1], self.eps,  n)
        S[0, n] = Q_f
        v[0, n] = m.doMulti(Q_f, self.d.transformX(np.subtract(self.x[n], self.x_f)))

        for i in range(1, n + 1):
            curr = n - i
            u_i = self.u[curr]
            v_t = self.d.transformU(u_i)
            x_i = self.x[0, curr]
            z_t = self.d.transformX(x_i)
            [A, B] = self.d.linearize(x_i, u_i, self.eps, curr)
            [Q, R] = self.c.approximate(x_i, u_i, self.eps, curr)
            [A_i, B_i, Q_i, R_i] = [m.convertToMatrix(A), m.convertToMatrix(B),
                                    m.convertToMatrix(Q), m.convertToMatrix(R)]
            #print ("A_i: " + str(A_i))
            #print ("B_i: " + str(B_i.shape))
            #print ("B_i: " + str(B_i.transpose()))
            #print ("Q_i: " + str(Q_i))
            #print ("R_i: " + str(R_i))
            #print ("S[0, curr+1]: " + str(S[0, curr+1]))
            #print ("B_i: " + str(m.transposeAll(B_i)))
            #print ("multiply: " + str(m.doMulti(m.transposeAll(B_i), S[0, curr+1])))
            # really hacky....fix this someday
            K1 = m.doMulti(m.doMulti(m.transposeAll(B_i), S[0, curr+1]), B_i) + R_i
            #print ("multi: " + str((m.doMulti(m.doMulti(m.transposeAll(B_i), S[0, curr+1]), B_i))))
            #print ("R1: " + str(R_i))
            print ("K1: " + str(K1))
            K1[0,0] = np.linalg.inv(K1[0,0])
            K2 = m.doMulti(m.doMulti(m.transposeAll(B_i), S[0, curr+1]), A_i)
            K[0, curr] = m.doMulti(K1, K2)
            K_v[0, curr] = m.doMulti(K1, m.transposeAll(B_i))
            K_u[0, curr] = m.doMulti(K1, R_i)

            #S1 = A_i[0] - m.doMulti(B_i, K[0, curr])
            
            S1 = m.convertToMatrix(A_i) - m.doMulti(B_i, K[0, curr])
            #print ("S1: " + str(S1))
            S[0, curr] = m.doMulti(m.doMulti(m.transposeAll(A_i), S[0, curr+1]), S1) + Q_i
            v[0, curr] = m.doMulti(m.transposeAll(S1), v[0, curr+1]) -\
                m.doMulti(m.doMulti(m.transposeAll(K[0, curr]), R_i), v_t) + m.doMulti(Q_i, z_t)

            du[0, curr] = (-1* m.doMulti(K[0, curr], self.d.transformX(dx[curr]))) -\
                m.doMulti(K_v[0, curr], v[0, curr+1]) -\
                m.doMulti(K_u[0, curr], v_t)

        return du
     
                
class Cost:
    def __init__(self):
	pass

    def solve(self, x_t, u_t, t):
	return 0

    def approximate(self, x_t, u_t, eps, t):
        Q_t = np.array([[np.eye(x_t.shape[1]), np.zeros(shape=(x_t.shape[1], x_t.shape[1]))],
                             [np.zeros(shape=(x_t.shape[1], x_t.shape[1])), np.eye(x_t.shape[1])]], 
                       dtype=object)

        R_t = np.array([[np.eye(u_t.shape[1])]])
        return [Q_t, R_t] 	

    def approximate_real(self, x_t, u_t, t):
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


class Dynamics:
    def __init__(self):
        pass

    def solve(self, X, U):
	dx = U[0,0] * math.cos(X[0,2])
	dy = U[0,0] * math.sin(X[0,2])
	dt = U[0,0] * math.tan(U[0,1])
	N = np.matrix([dx, dy, dt])
		
        return N

    def getTrajectory(self, x0, u):
        self.x0 = x0
        self.u0 = u[0,0]
        #print str(u.shape)
        traj = np.zeros(shape=(u.shape[0] + 1, x0.shape[0], x0.shape[1]), dtype=object)
        traj[0] = x0
        #print ("traj: " + str(traj))
        for i in range(u.shape[0]):
            #print ("u[0, i]: " + str(u[i]))
            traj[i+1] = self.solve(traj[i], u[i]) + traj[i]
        
        self.setCurrentTrajectory(traj, u)
        #print ("end traj: " + str(traj))
        return traj

    def setCurrentTrajectory(self, x, u):
        self.xs = x
        self.us = u

    def getCurrentState(self, t):
        return self.xs[0, t]

    def getCurrentControl(self, t):
        return self.us[t]

    def transformX(self, x):
        return np.array([[np.matrix(x).transpose()], [np.matrix([[1],[1],[1]])]])

    def transformU(self, u):
        return np.array([[u.transpose()]])

    def linearize(self, x_t, u_t, eps, t):
        
        def computeA(x_t, u_t, eps, t):
            A_j = np.zeros(shape=(x_t.shape[1], x_t.shape[1]))

            for i in range(x_t.shape[1]):
                A_j[i] = m.getFiniteDiff1(self, x_t, u_t, eps, i, True)

            a_12 = np.zeros(shape=A_j.shape)
            a = self.solve(x_t, u_t) - self.getCurrentState(t+1)
            np.fill_diagonal(a_12, a)
            a_21 = np.dot(0, A_j)
            a_22 = np.eye(A_j.shape[0])

            A_t = np.array([[A_j.transpose(), a_12],
                             [a_21, a_22]], dtype=object)
            return A_t

        def computeB(x_t, u_t, eps, t):
            B_j = np.zeros(shape=(u_t.shape[1], x_t.shape[1]))

            for i in range(u_t.shape[1]):
                B_j[i] = m.getFiniteDiff1(self, x_t, u_t, eps, i, False)

            B_t = np.array([[B_j.transpose()], 
                             [np.zeros(shape=B_j.transpose().shape)]], dtype=object)

            return B_t

        return [computeA(x_t, u_t, eps, t), computeB(x_t, u_t, eps, t)]


class LQRMath:
    def __init__(self):
        pass

    def getDelta(self, var, eps):
	return var * eps

    def getFiniteDiff1(self, f, x, y, eps, index, firstTerm):
	if firstTerm:
            d_x = np.zeros(shape=x.shape)
	    d_x[0, index] = self.getDelta(x[0,index], eps)
 	    top = (f.solve(x + d_x, y) -\
                       f.solve(x - d_x, y))
            bottom = 2 * d_x
            
	else:
            d_y = np.zeros(shape=y.shape)
	    d_y[0, index] = self.getDelta(y[0,index], eps)
	    top = (f.solve(x, y + d_y) -\
                       f.solve(x, y - d_y))
            bottom = 2 * d_y

        top = top / bottom[0,index]
        
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
    
    def transposeAll(self, M):
        N = np.zeros(shape=M.transpose().shape, dtype=object)
        for i in range(M.shape[0]):
            for j in range(M.shape[1]):
                if isinstance(M[i, j], np.ndarray):
                    N[j, i] = M[i, j].transpose()
                else:
                    N[j, i] = M[i, j]
        return N

    def doMulti(self, M, N):
        r = M.shape[0]
        c = N.shape[1]
        L = np.zeros(shape=(r, c), dtype=object)
        for i in range(r):
            #print r
            for j in range(c):
                #print c
                #print(N.shape[0])
                for k in range(N.shape[0]):
                    #print ("M[i,k]: " + str(M[i,k]))
                    #print ("N[k,j]: " + str(N[k,j]))
                    #print ("L[i,j]: " + str(L[i,j]))
                    
                    L[i, j] += np.dot(M[i, k], N[k, j])

        #print ("L: " + str(L))

        return L

    def convertToMatrix(self, N):
        r = N.shape[0]
        c = N.shape[1]
        M = np.eye(r)
        L = np.zeros(shape=(r, c), dtype=object)
        for i in range(r):
            #print r
            for j in range(c):
                #print c
                #print(N.shape[0])
                for k in range(N.shape[0]):
                    #print ("M[i,k]: " + str(M[i,k]))
                    #print ("N[k,j]: " + str(N[k,j]))
                    #print ("L[i,j]: " + str(L[i,j]))
                    
                    L[i, j] += np.dot(M[i, k], N[k, j])

        #print ("L: " + str(L))

        return L

def main():
    np.set_printoptions(suppress=True, precision=4)

    d = Dynamics()
    x_0 = np.matrix([1, 1, math.pi/3.0])
    x_f = np.matrix([20, 15, math.pi/6.0])
    c = Cost()
    u = np.array([np.matrix([3, math.pi/6.0]), np.matrix([-1, 0]), np.matrix([4, math.pi/4.0]),
                   np.matrix([-2.5, -math.pi/3.0]), np.matrix([-2, math.pi/9.0]),
                   np.matrix([3, math.pi/6.0]), np.matrix([-5, 0]), np.matrix([3, math.pi/6.0])],
                 dtype=object)
    dt = u.shape[1] - 1
    its = 30
    tol = 10 ** -6
    
    lqr = LQRController(d, x_0, x_f, c, u, dt, its, tol)
    u = lqr.runiLQR()
    print (str(u))
    
    '''x_init = np.matrix([1, 1, math.pi/3.0])
    u_init = np.matrix([3, math.pi/6.0])
    d = Dynamics(x_init, u_init)
    lqr = LQRController(d)

    xGuess = [x_init, d.solve(x_init, u_init)]
    print("Solution: " + str(d.solve(x_init, u_init)))
    uGuess = [u_init, np.matrix([1, math.pi/4])]

    d.setCurrentTrajectory(xGuess, uGuess)

    [A,B] = d.linearize(x_init, u_init, 10.0**-6, 0)
    #print ("A: " + str(A[1,0]))
    #print ("B: " + str(B))
    #print (str(np.matrix([[x_init.transpose()], [[1], [1], [1]]])))
    #print (str(np.array([[x_init.transpose()], [np.matrix([[1],[1],[1]])]])))
    #print ("B[0,0]: " + str(B[0,0]))
    #print ("A.t: " + str(m.transposeAll(A)))

    D = np.dot(A, np.matrix([[x_init.transpose()], [1]])) +\
                  np.matrix([[np.dot(B[0,0], u_init.transpose())],
                             [0]])

    D = m.doMulti(A, np.array([[x_init.transpose()], [np.matrix([[1],[1],[1]])]])) +\
         m.doMulti(B, np.array([[u_init.transpose()]]))

    print ("D: " + str(D))'''
    



if __name__ == '__main__':
    m = LQRMath()
    main()
