from framework import *
import random
import math
import numpy as np

class RotatingTable(Framework):
	name = "Rotating Table"
	description = "First Iteration on building a rotating table"
	boxes = []
	joints = []

	def __init__(self):
		super(RotatingTable, self).__init__()

		# Set the gravity to be 0 in alll direction
		self.world.gravity = (0,0)

		self.center_pos_ = (0,18)

		self.radius = 18.0

		# Build the edge of a table by connecting multiple edge lines
		table_edge = self.world.CreateStaticBody(position=(0.0,0.0))
		
		# Circle table
		table_edge.CreateEdgeChain(self.makeCircleTableVertices(4))

		# Build the center of the table
		circle = b2FixtureDef(shape=b2CircleShape(radius=0.2), friction=0.9)
		table_center = self.world.CreateStaticBody(position=self.center_pos_, fixtures=circle)

		# Build the obstruction objects in the table
		box = b2FixtureDef(shape=b2PolygonShape(box=(0.5,0.5)), density=5, friction=0.9)

		# Build the target object in the table
		circle_target = b2FixtureDef(shape=b2CircleShape(radius=1.0))

		# inside_table = [(x,y) for x in range(0,30) for y in range(15,30)]
		inside_table = [(x, y) for x in range(-8,8) for y in range(0,30)]
		random.shuffle(inside_table)

		# Create the obstruction object to the world
		num_obs_object = 10
		self.bodies = [self.world.CreateDynamicBody(position=pos, fixtures=box) for pos in inside_table[:num_obs_object]]

		# Create the target object to the world
		self.bodies.append(self.world.CreateDynamicBody(position=inside_table[num_obs_object], fixtures=circle_target))

	def rotate_cw(self, body):
		fixture = b2FixtureDef(shape=body.fixtures[0].shape, density=5, friction=0.9)

		theta = math.radians(-3.0)
		rotation_matrix = np.asmatrix([[math.cos(theta), -1*math.sin(theta)],
						   			   [math.sin(theta), math.cos(theta)   ]])
		new_body_position = rotation_matrix * (np.asmatrix(body.position) - np.asmatrix(self.center_pos_)).T + np.asmatrix(self.center_pos_).T
		new_body_position = np.ravel(new_body_position.T)
		return self.world.CreateDynamicBody(position=new_body_position,
											fixtures=fixture,
											angle=body.angle,
											angularVelocity=body.angularVelocity,
											linearVelocity=body.linearVelocity)

	def rotate_ccw(self, body):
		fixture = b2FixtureDef(shape=body.fixtures[0].shape, density=5, friction=0.9)
		theta = math.radians(3.0)
		rotation_matrix = np.asmatrix([[math.cos(theta), -1*math.sin(theta)],
						   			   		[math.sin(theta), math.cos(theta)   ]])
		new_body_position = rotation_matrix * (np.asmatrix(body.position) - np.asmatrix(self.center_pos_)).T + np.asmatrix(self.center_pos_).T
		new_body_position = np.ravel(new_body_position.T)
		return self.world.CreateDynamicBody(position=new_body_position,
											fixtures=fixture,
											angle=body.angle,
											angularVelocity=body.angularVelocity,
											linearVelocity=body.linearVelocity)

	def rotate_cw_all(self):
		"""
		Rotate all of the objects with respect to the center of the table in clockwise direction
		"""
		# Apply rotation matrix about the center point to all of the objects and destroy all the previous object
		new_bodies = []
		for body in self.bodies:
			new_bodies.append(self.rotate_cw(body))
			self.world.DestroyBody(body)
		self.bodies = new_bodies

	def rotate_ccw_all(self):
		"""
		Rotate all of the objects on the table with respect to the center of the table in counter-clockwise direction
		"""
		new_bodies = []
		for body in self.bodies:
			new_bodies.append(self.rotate_ccw(body))
			self.world.DestroyBody(body)
		self.bodies = new_bodies

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
		starting_point = (circle_center_pos[0], circle_center_pos[1] + self.radius)
		vertices = [starting_point]
		for i in range(num_sides):
			last_pos = vertices[-1]
			new_vertex = rotation_matrix * (np.asmatrix(last_pos) - np.asmatrix(circle_center_pos)).T + np.asmatrix(circle_center_pos).T
			vertices.append(tuple(np.ravel(new_vertex.T)))
		print circle_center_pos
		print vertices
		return [(x,y) for x,y in vertices]

	def Keyboard(self, key):
		if key==Keys.K_s:
			self.rotate_cw_all()
		elif key==Keys.K_w:
			self.rotate_ccw_all()

if __name__ == "__main__":
	main(RotatingTable)



