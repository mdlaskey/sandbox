import numpy as np
import numpy.linalg as la
import collections
import cgt
from cgt import nn
import IPython
import tensorflow as tf

class Policy_Gradient:

	""" 
	Calculates policy gradient
	for given input state/actions.
	"""

	def __init__(self, goal_state=None):
		self.goal_state = goal_state
		self.gradient, self.net_func, self.net_weights, self.net = self.init_neural_net()


	def estimate_q(self, traj, rewards):
		"""
		Estimates the q-values for a trajectory
		based on the intermediate rewards.

		Parameters:
		traj: array-like
			List of state-action pairs.

		rewards: array-like
			List of rewards.

		Output: 
		q: array-like
			Estimated q-values.

		q_dict: dict
			Dictionary of q-value, indexed by state
		"""
		# Converts np arrays to tuples, for dictionary hashing
		traj = map(lambda t: (tuple(list(np.ravel(t[0]))), tuple(list(np.ravel(t[1])))), traj)
		print "traj:\n{}\n".format(traj)
		count = collections.Counter()
		for sa_pair in traj:
			count[sa_pair] += 1
		print "Rewards: \n{}\n".format(rewards)
		q_dict = {traj[i]: rewards[i] * count[traj[i]] for i in range(len(traj))}
		q = np.array([q_dict[sa_pair] for sa_pair in traj])
		return q, q_dict


	def gradient_update(self, traj, rewards, stepsize=0.01):
		"""
		Estimates and applies gradient update according to a policy.

		Parameters:
		traj: array-like
			List of state-action pairs.

		rewards: array-like
			List of rewards.

		stepsizeval: float
			Step size.

		Output: 
			None.
		"""
		# Estimate q-values and extract gradients
		q, q_dict = self.estimate_q(traj, rewards)
		assert len(q) == len(traj)
		# traj = np.array(traj)
		# Calculate updates and create update pairs
		traj = np.array(traj)
		input_obs = [traj[i][0].T for i in range(len(traj))]
		a_na = [traj[i][1].T for i in range(len(traj))]
		grad_values = [self.gradient(input_obs[i], a_na[i], np.array([q[i]])) for i in range(len(traj))]
		# print traj[:,0]
		# print traj[:,1]
		# print q
		# grad_values = [self.gradient(traj[:,0], traj[:,1], q) for i in range(len(traj))]
		print "self.net_weights", self.net_weights[0], self.net_weights[0].shape
		print "grad_values", grad_values[0], grad_values[0].shape
		print len(self.net_weights)
		print len(grad_values)
		IPython.embed()

		# Sum gradients
		updates = [(param, param - stepsize * steps) for (param, steps) in zip(self.net_weights, grad_values)]    

		# Create and call updater function
		updater = cgt.function([], self.net, updates=updates)
		updater()

		return None


	def init_neural_net(self):
		"""
		Sets up neural network for policy gradient.

		Output:
		logprobs: array-like
			Output of neural network.

		params: array-like
			Weights of neural network.

		net_func: CGT function
			Function of output layer.

		log_net_func: CGT function
			Function of log of output layer.
		"""
		# Input
		input_obs_shape = (1, 2)
		self.ctrl_dim = input_obs_shape[1]

		# Placeholders
		# input_obs = cgt.matrix("input_obs", fixed_shape = (None, 2))
		# a_na = cgt.matrix("a_na",fixed_shape = (None, self.ctrl_dim))
		input_obs = cgt.matrix("input_obs")
		a_na = cgt.matrix("a_na")
		q_n = cgt.vector("q_n")

		# Standard Deviations
		logstd_1a = nn.parameter(np.zeros((1, self.ctrl_dim)), name="std_1a")
		std_1a = cgt.exp(logstd_1a)
		b = cgt.size(input_obs, 0)
		std_na = cgt.repeat(std_1a, b, axis=0)
		
		# Net
		mean_na = nn.Affine(2, 2)(input_obs)
		logp_n = ((-.5) * cgt.square( (a_na - mean_na) / std_na ).sum(axis=1)) - logstd_1a.sum()
		surr = (logp_n*q_n).mean()
		params = nn.get_parameters(surr)
		print params
		gradient = cgt.function([input_obs, a_na, q_n], \
			cgt.concatenate([p.flatten() for p in cgt.grad(surr,params)]))

		# Outputs
		pdists_np = cgt.concatenate([mean_na, std_na], axis=1)
		net_func = cgt.function([input_obs], pdists_np)

		# Net Weights
		net_weights = nn.get_parameters(surr)
		print "net weights", [x.shape for x in net_weights]
		return gradient, net_func, net_weights, mean_na

		# Network
		# d0,d1,d2,d3 = input_obs.shape
		# flatlayer = input_obs.reshape([d0,d1*d2*d3])
		# flatlayer_shape = cgt.infer_shape(flatlayer)[1]

		# Batch Size input_obs Channels input_obs Width input_obs Height
		# Xshape = (1, 3, 32, 32)
		# input_obs = cgt.tensor4("input_obs", fixed_shape = Xshape)
		# y = cgt.vector("y", fixed_shape = (batchsize,), dtype='i4')

		# # Network
		# # conv1 = nn.SpatialConvolution(3, 32, kernelshape=(11,11), pad=(2,2))(input_obs)
		# # d0,d1,d2,d3 = conv1.shape
		# flatlayer = conv1.reshape([d0,d1*d2*d3])
		# nfeats = cgt.infer_shape(flatlayer)[1]
		# ip1 = nn.Affine(nfeats, 40)(input_obs)
		# relu1 = nn.rectify(ip1)

		# d0,d1,d2,d3 = relu1.shape
		# flatlayer = relu1.reshape([d0,d1*d2*d3])
		# nfeats = cgt.infer_shape(flatlayer)[1]
		# ip2 = nn.Affine(nfeats, 8)(flatlayer) 
		# relu2 = nn.rectify(ip2)

		# # Output of network (softmax probabilities)
		# logprobs = nn.logsoftmax(relu2)

		# # Parameters of network
		# params = nn.get_parameters(logprobs)

		# return logprobs, params

	def get_action(self, state):
		"""
		Returns action based on input state.

		Input:
		state: array-like
			Input state.

		Output:
		action: array-like
			Predicted action.
		"""
		flattened = state.T
		output_pdist = self.net_func(flattened)
		action = meanstd_sample(output_pdist)
		return action.T
		# print flattened
		# print flattened.shape
		# print self.net_func
		# print self.net_func(flattened)
		# print self.net_func(flattened).shape

def meanstd_sample(meanstd_np):
	"""
	Samples an action based on
	the input probability distribution.
	"""
	d = meanstd_np.shape[1]//2
	mean = meanstd_np[:,0:d]
	std = meanstd_np[:,d:2*d]
	return mean + std * np.random.randn(*std.shape)
