import numpy as np
import numpy.linalg as la
import collections
import cgt

class Policy_Gradient:

	""" 
	Calculates policy gradient
	for given input state/actions.
	"""

	def __init__(self, opt_grasp_coords=None):
		self.opt_grasp_coords = opt_grasp_coords

	def reward_function(self, s, a):
		"""
		Calculates rewards for a state-action pair.

		Parameters:
		s: state object
			Input state.

		a: array-like
			Input action.

		Output:
		r: float
			Reward, defined according to reward function.
		"""
		dist_reward = 1.0 / la.norm(self.opt_grasp_coords - s.grasp_coords)
		obj_penalty = np.sum(s.out_of_bounds())

		return dist_reward + obj_penalty


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
		q: dict
			Estimated q-values.
		"""
		count = collections.Counter()
		for sa_pair in traj:
			count[sa_pair] += 1
		q_dict = {traj[i]: rewards[i] * count[traj[i]] for i in range(len(traj))}
		return q_dict


	def estimate_gradient(self, pi, theta, traj, rewards):
		"""
		Estimates gradient according to a policy.

		Parameters:
		pi: function
			Policy at current iteration.

		theta: array-like
			Parameters that characterize the policy.

		traj: array-like
			List of state-action pairs.

		rewards: array-like
			List of rewards.

		Output: 
		grad: array-like
			Policy gradient.
		"""
		policy = pi(theta)
		q_dict = self.estimate_q(traj, rewards)


