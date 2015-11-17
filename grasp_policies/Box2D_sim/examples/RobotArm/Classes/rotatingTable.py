from framework import *
import random
import math
import numpy as np

class RotatingTable(Framework):
	name = "Rotating Table"
	description = "First Iteration on building a rotating table"
	boxes = []
	joints = []

	def __init__(self, offset, center, radius, world = None):
		super(RotatingTable, self).__init__()

		# Set the gravity to be 0 in alll direction
		self.world.gravity = (0,0)

		self.center_pos_ = (offset,center)

		self.radius_ = radius

		self.num_obs_object_ = 4

		self.color_ = b2Color(0.9,0.9,0.4)

		self.lin_damp_ = 5

		self.ang_damp_ = 5

		if world != None:
			self.parent_world_ = world
			print world.contactCount
		else:
			self.parent_world_ = self.world


	def rotate_cw(self, body):
		fixture = b2FixtureDef(shape=body.fixtures[0].shape, density=5, friction=0.9)

		theta = math.radians(-3.0)
		rotation_matrix = np.asmatrix([[math.cos(theta), -1*math.sin(theta)],
						   			   [math.sin(theta), math.cos(theta)   ]])
		if (self.checkInBounds(body)):
			new_body_position = rotation_matrix * (np.asmatrix(body.position) - np.asmatrix(self.center_pos_)).T + np.asmatrix(self.center_pos_).T
			new_body_position = np.ravel(new_body_position.T)
		else:
			new_body_position = body.position

		return self.parent_world_.CreateDynamicBody(position=new_body_position,
											fixtures=fixture,
											angle=body.angle,
											angularVelocity=body.angularVelocity,
											linearVelocity=body.linearVelocity,
											linearDamping=self.lin_damp_,
											angularDamping=self.ang_damp_)

	def rotate_ccw(self, body):
		fixture = b2FixtureDef(shape=body.fixtures[0].shape, density=5, friction=0.9)
		theta = math.radians(3.0)
		rotation_matrix = np.asmatrix([[math.cos(theta), -1*math.sin(theta)],
						   			   		[math.sin(theta), math.cos(theta)   ]])
		new_body_position = rotation_matrix * (np.asmatrix(body.position) - np.asmatrix(self.center_pos_)).T + np.asmatrix(self.center_pos_).T
		new_body_position = np.ravel(new_body_position.T)

		if (self.checkInBounds(body)):
			new_body_position = rotation_matrix * (np.asmatrix(body.position) - np.asmatrix(self.center_pos_)).T + np.asmatrix(self.center_pos_).T
			new_body_position = np.ravel(new_body_position.T)
		else:
			new_body_position = body.position

		return self.parent_world_.CreateDynamicBody(position=new_body_position,
											fixtures=fixture,
											angle=body.angle,
											angularVelocity=body.angularVelocity,
											linearVelocity=body.linearVelocity,
											linearDamping=self.lin_damp_,
											angularDamping=self.ang_damp_)

	def rotate_cw_all(self):
		"""
		Rotate all of the objects with respect to the center of the table in clockwise direction
		"""
		# Apply rotation matrix about the center point to all of the objects and destroy all the previous object
		new_bodies = []
		for body in self.bodies:
			new_bodies.append(self.rotate_cw(body))
			self.parent_world_.DestroyBody(body)
		self.bodies = new_bodies

	def rotate_ccw_all(self):
		"""
		Rotate all of the objects on the table with respect to the center of the table in counter-clockwise direction
		"""
		new_bodies = []
		for body in self.bodies:
			new_bodies.append(self.rotate_ccw(body))
			self.parent_world_.DestroyBody(body)
		self.bodies = new_bodies

	def checkInBounds(self, body):
		"""
		Checks whether an object is inside the rotational table or num_obs_object_

		@param body : the object of interest

		@return boolean that indicates if the object is inside the table
		"""
		pos = body.position
		dist_from_center = math.sqrt((self.center_pos_[0] - pos[0]) ** 2 + (self.center_pos_[1] - pos[1]) ** 2)
		if dist_from_center > self.radius_:
			return False
		return True

	def marginFromEdge(self, body):
		"""
		Calculate how far the object is from the edge of the table

		@param body : the object of interest

		@return margin : a double
		""" 
		pos = body.position
		dist_from_center = math.sqrt((self.center_pos_[0] - pos[0]) ** 2 + (self.center_pos_[1] - pos[1]) ** 2)
		margin = self.radius_ - dist_from_center
		return margin

	def makeCircleTableVertices(self, num_sides):
		"""
		Create the vertices coordinate of a circle approximation using polygon

		@param num_sides : int, the number of polygon side, the bigger the number, the closer it is to a circle
			
		@return list of vertices
		"""

		# Assign the center position of the circle
		circle_center_pos = self.center_pos_

		# How accurate we want to approximate the circle table (use num of sides)
		theta = 2 * math.pi / num_sides
		rotation_matrix = np.asmatrix([[math.cos(theta), -1*math.sin(theta)],
						   			   		[math.sin(theta), math.cos(theta)   ]])

		# starting point signifies the start place we want to draw the circle approximation
		starting_point = (circle_center_pos[0], circle_center_pos[1] + self.radius_)
		# nxt = (11.0, self.radius_)
		# return [starting_point, nxt]
		vertices = [starting_point]
		for i in range(num_sides):
			last_pos = vertices[-1]
			new_vertex = rotation_matrix * (np.asmatrix(last_pos) - np.asmatrix(circle_center_pos)).T + np.asmatrix(circle_center_pos).T
			vertices.append(tuple(np.ravel(new_vertex.T)))
		return [(x,y) for x,y in vertices]

	def getBoxes(self):
		"""
		Get the obstacle bodies of the boxes as a b2Body object

		@return list of b2Body
		"""
		return self.bodies[:len(boxes) - 1]

	def getTarget(self):
		"""
		Get the body of the boxes as a b2Body object

		@return b2Body
		"""
		return self.bodies[-1]

	def getBoxesState(self):
		"""
		return the features of each box as an array-like 2D matrix.

		the feature in order : (position, angle, linear velocity, angular velocity)
		"""
		features = []
		for box in self.bodies[:-1]:
			# Extract the feature of each box and combine them as an array
			features.append(np.ndarray([box.position, box.angle, box.linearVelocity, box.angularVelocity]))
		return np.vstack(features)

	def getTargetState(self):
		"""
		return the features of the target object as an array-like object

		the feature in order : (position, angle, linear velocity, angular velocity)
		"""
		features = []
		target = box[-1]
		# Extract the feature of each box and combine them as an array
		features.append(np.ndarray([target.position, target.angle, target.linearVelocity, target.angularVelocity]))
		return np.vstack(features)

	def Keyboard(self, key):
		if key==Keys.K_s:
			self.rotate_cw_all()
		elif key==Keys.K_w:
			self.rotate_ccw_all()

	def addToWorld(self):
		"""
		Add the physical bodies of all the obstacle objects and target objects to the world where this object is
		instantiated
		"""

		# Build the obstruction objects in the table
		box = b2FixtureDef(shape=b2PolygonShape(box=(1.1,1.1)), density=5, friction=0.9)

		# Build the target object in the table
		circle_target = b2FixtureDef(shape=b2CircleShape(radius=1.1))

		# inside_table = [(x,y) for x in range(0,30) for y in range(15,30)]
		# inside_table = [(x, y) for x in range(int(self.center_pos_[0] - self.radius_ / 2.5), int(self.center_pos_[0] + self.radius_ / 2.5)) for y in range(int(self.center_pos_[1] - self.radius_), int(self.center_pos_[1] + self.radius_))]
		inside_table = [(np.random.normal(loc=self.center_pos_[0],scale=self.radius_/3.0), np.random.normal(loc=self.center_pos_[1],scale=self.radius_/3.0)) for _ in range(self.num_obs_object_+1)]
		# Create the obstruction object to the world
		self.bodies = [self.parent_world_.CreateDynamicBody(position=pos,
															fixtures=box,
															linearDamping=self.lin_damp_,
															angularDamping=self.ang_damp_)
															for pos in inside_table[:self.num_obs_object_]]

		# Create the target object to the world
		self.bodies.append(self.parent_world_.CreateDynamicBody(position=inside_table[self.num_obs_object_],
																fixtures=circle_target,
																linearDamping=self.lin_damp_,
																angularDamping=self.ang_damp_))


