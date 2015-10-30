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

		self.center_pos_ = (0,15)

		# Build the edge of a table by connecting multiple edge lines
		table_edge = self.world.CreateStaticBody(position=self.center_pos_)
		# temporary square table
		table_edge.CreateEdgeChain([(-15,-15),
								    (-15,15),
								    (15,15),
								    (15,-15),
								    (-15,-15)	
									])
		# table.edge.CreateEdgeChain([(0,0),
		# 							()])

		# Build the center of the table
		circle = b2FixtureDef(shape=b2CircleShape(radius=1.0))
		table_center = self.world.CreateStaticBody(position=self.center_pos_, fixtures=circle)

		# Build the objects in the table
		box = b2FixtureDef(shape=b2PolygonShape(box=(0.5,0.5)), density=5, friction=0.2)

		# inside_table = [(x,y) for x in range(0,30) for y in range(15,30)]
		inside_table = [(x, y) for x in range(-11,11) for y in range(0,30)]
		random.shuffle(inside_table)

		self.bodies = [self.world.CreateDynamicBody(position=pos, fixtures=box) for pos in inside_table[:10]]

	def rotate_cw(self, body):
		box = b2FixtureDef(shape=b2PolygonShape(box=(0.5,0.5)), density=5, friction=0.2)
		theta = math.radians(3.0)
		rotation_matrix = np.asmatrix([[math.cos(theta), -1*math.sin(theta)],
						   			   [math.sin(theta), math.cos(theta)   ]])
		new_body_position = rotation_matrix * (np.asmatrix(body.position) - np.asmatrix(self.center_pos_)).T + np.asmatrix(self.center_pos_).T
		new_body_position = np.ravel(new_body_position.T)
		return self.world.CreateDynamicBody(position=new_body_position, fixtures=box)

	def rotate_ccw(self, body):
		box = b2FixtureDef(shape=b2PolygonShape(box=(0.5,0.5)), density=5, friction=0.2)
		theta = 3.0
		rotation_matrix = -1 * np.asmatrix([[math.cos(theta), -1*math.sin(theta)],
						   			   		[math.sin(theta), math.cos(theta)   ]])
		new_body_position = rotation_matrix * (np.asmatrix(body.position) - np.asmatrix(self.center_pos_)).T + np.asmatrix(self.center_pos_).T
		new_body_position = np.ravel(new_body_position.T)
		return self.world.CreateDynamicBody(position=new_body_position, fixtures=box)

	def rotate_cw_all(self):
		"""
		Rotate all of the objects with respect to the center of the table in clockwise direction
		"""
		# Apply rotation matrix about the center point to all of the objects and destroy all the previous object
		new_bodies = []
		for body in self.bodies:
			self.world.DestroyBody(body)
			new_bodies.append(self.rotate_cw(body))
		self.bodies = new_bodies

	def rotate_ccw_all(self):
		"""
		Rotate all of the objects on the table with respect to the center of the table in counter-clockwise direction
		"""
		new_bodies = []
		for body in self.bodies:
			self.world.DestroyBody(body)
			new_bodies.append(self.rotate_ccw(body))
		self.bodies = new_bodies

	def Keyboard(self, key):
		if key==Keys.K_w:
			self.rotate_cw_all()
		elif key==Keys.K_s:
			self.rotate_ccw_all()


if __name__ == "__main__":
	main(RotatingTable)


