from test_one_armed_robot import GraspingWorld
import settings
import numpy as np
import numpy.linalg as la
import numdifftools as nd
import math
import IPython
from framework import *

# Global Variables: 
# This is your measured data from Real World Zek. U is the control applied and X is the distance moved
# Note this is for one DOF of the arm, you will need to tune each one separately 

# U : ndarray with control as its elements
# X : 
# [U,X] = Data

# This is the parameters in the robot arm itself, you mention these will be damping, mass, etc.
# Parameters
# 0 Angular Damping (angular_damp)
# 1 Linear Damping (linear_damp)
# 2 MassDensity (mass)
# 3 Friction (fric)
# 4 Rotating Table Motor Speed (rtms)
# 5 Rotating Arm Motor Speed (rams)
# 6 Translating Arm Motor Speed (tams)
# 7 Translating Gripper Finger Motor Speed (tgms)
# 8 Rotating Arm maxMotorTorque (ra_torque)
# 9 Translating Arm maxMotorForce (ta_force)
# 10 Translating Gripper Finger maxMotorForce (tg_force)

init_theta = [3.0, 7.0, 5.0, 1.0, 10.0, 15.0, 20.0, 20.0, 5.0, 7.0, 100.0]
big_theta = [100.0, 60.0, 90.0, 30.0, 500.0, 900.0, 80.0, 90.0, 120.0, 500.0, 90.0]
# Controls :
# Rotating Table
# Rotating Arm
# Translation of Arm
# Translation of Gripper

U = np.array([np.array([0.0, 0.0, 0.0, -11.0]),
				np.array([0.0, 20.0, -70.0, 77.0])])

X = np.array([np.array([1.0, 7.0, 1.0, -5.0, 4.0, 10.0]),
				np.array([6.0, 3.0, -2.0, 9.0, -1.0, -2.0])])

def Loss(theta):
	# Number of Data points collected
	N = U.shape[0]
	loss_sum = 0
	for i in range(N):
		loss_sum = la.norm(X[i,:] - evalSim(U[i,:],theta))
	return loss_sum/N

def evalSim(u,theta): 
	#Call box2d with control and theta parameters, return the distance moved
	#TODO: Implement this 

	sim = GraspingWorld(False, None, False, False, None, internal_params = theta)
	
	initial_state = sim.getState()
	print initial_state

	# Apply Control to the Simulation
	sim.applyControl(u[0], u[1], u[2], u[3])

	# Decide on the fps the simulation will work on
	settings = fwSettings
	sim.Step(settings)
	final_state = sim.getState()
	print final_state

	# Calculate the difference of each entry of the final state and the initial state
	# """
 #        col #       State
 #        0           moving arm 1 position
 #        1           moving arm 2 position
 #        2           extension arm position
 #        3           Gripper's Palm position
 #        4           Gripper's left finger position
 #        5           Gripper's right finger position
 #        6           Angle of Rotational Joint

 #        7           Target Object position
 #        8           Target Object angle
 #        9           Target Object linear velocity
 #        10          Target Object angular velocity
 #    """
	
	diff_vector = []
	for i in range(final_state.shape[0]-1):
		diff_x = final_state[i][0] - initial_state[i][0]
		diff_y = final_state[i][1] - initial_state[i][0]
		diff_vector.append(np.sqrt(diff_y**2 + diff_x**2))

	return np.array(diff_vector)


# We are using finite differences to compute the gradient with respect to the parameters 
def computeGrad(): 
	func = lambda theta: Loss(theta)
	grad = nd.Gradient(func,step = 5e-10)
	return grad

#This part is where we descend into theta
def updateParameters(theta):
	# TODO
	# You should look up line search for gradient descent here to calculate alpha

	alpha = 1

	old_loss = Loss(theta)
	print computeGrad()(theta)
	line_search_theta = theta - alpha*computeGrad()(theta)
	loss = Loss(line_search_theta)

	while (loss > old_loss):
		alpha *= 0.9
		line_search_theta = theta - alpha*computeGrad()(theta)
		loss = Loss(line_search_theta)
	return line_search_theta

#Final part is to now repeate the process until we converge 
def tuneArm(): 
	epsilon = 1e-4
	loss_prev = Loss(init_theta)
	theta = updateParameters(init_theta)
	loss = Loss(theta)
	while(loss_prev - loss> epsilon):
		loss_prev = Loss(theta)
		theta = updateParameters(theta)
		loss = Loss(theta)
	return theta

# theta = updateParameters(init_theta)
# theta = init_theta
# for i in range(2):
# 	theta = updateParameters(theta)
# print Loss(init_theta)
# print Loss(theta)
print evalSim(U[1,:], init_theta)
print evalSim(U[1,:], big_theta)