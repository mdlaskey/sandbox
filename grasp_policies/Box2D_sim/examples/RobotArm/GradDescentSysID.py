import numpy as np
import numpy.linalg as la
import numdifftools as nd
import math
import IPython

# Global Variables: 
# This is your measured data from Real World Zek. U is the control applied and X is the distance moved
# Note this is for one DOF of the arm, you will need to tune each one separately 

# U : ndarray with control as its elements
# X : 
[U,X] = Data

# This is the parameters in the robot arm itself, you mention these will be damping, mass, etc.
# Angular Damping
# Linear Damping
# Mass
# 
theta = []

def Loss(theta):
	#Number of Data points collected
	N = U.shape[0]
	loss_sum = 0
	for i in range(N): 
		loss_sum = la.norm(X[i,:] - evalSim(U[i,:],theta))
	return loss_sum/N

def evalSim(u,theta): 
	#Call box2d with control and theta parameters, return the distance moved
	#TODO: Implement this 

	return x


#We are using finite differences to compute the gradient with respect to the parameters 
def computeGrad(): 
	func = lambda theta: Loss(theta)
	grad = nd.Gradient(func,step = 5e-1)

#This part is where we descend into theta
def updateParameters():
	grad = computeGrad()
	theta = theta - alpha*grad
	#TODO
	#You should look up line search for gradient descent here to calculate alpha 

	return theta

#Final part is to now repeate the process until we converge 
def tuneArm(): 
	epsilon = 1e-4
	loss_prev = Loss(theta)
	theta = updateParameters()
	loss = Loss(theta)
	while(loss_prev - loss> epsilon):
		loss_prev = Loss(theta)
		theta = updateParameters()
		loss = Loss(theta)

	return theta