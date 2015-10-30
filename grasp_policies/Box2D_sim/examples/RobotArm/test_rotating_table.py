from framework import *
import random
from math import sqrt

class RotatingTable(Framework):
	name = "Rotating Table"
	description = "First Iteration on building a rotating table"
	boxes = []
	joints = []

	def __init__(self):
		super(RotatingTable, self).__init__()

		# Set the gravity to be 0 in alll direction
		self.world.gravity = (0,0)

		# Build the edge of a table by connecting multiple edge lines
		table_edge = self.world.CreateStaticBody(position=(0, 15))
		# temporary square table
		table_edge.CreateEdgeChain([(-15,-15),
								    (-15,15),
								    (15,15),
								    (15,-15),
								    (-15,-15)	
									])

		# Build the center of the table
		circle = b2FixtureDef(shape=b2CircleShape(radius=2.0))
		table_center = self.world.CreateDynamicBody(position=(0, 15), fixtures=circle)

		# Build the objects in the table
		box = b2FixtureDef(shape=b2PolygonShape(box=(0.5,0.5)), density=5, friction=0.2)

		# inside_table = [(x,y) for x in range(0,30) for y in range(15,30)]
		inside_table = [(0,0), (1,3)]
		random.shuffle(inside_table)

		self.bodies = [self.world.CreateDynamicBody(position=pos, fixtures=box) for pos in inside_table[:2]]

		# Create the joints between the center of the table with all of the objects
        #         bodyA      bodyB   localAnchorA localAnchorB
		sets = [(table_center, body, (0,0), (0.0,0.0)) for body in self.bodies]

		self.joints=[]
		for bodyA, bodyB, localAnchorA, localAnchorB in sets:
			dfn=b2DistanceJointDef(
					frequencyHz=4.0,
					dampingRatio=0.5,
					bodyA=bodyA,
					bodyB=bodyB,
					localAnchorA=localAnchorA,
					localAnchorB=localAnchorB,
				)
			self.joints.append( self.world.CreateJoint(dfn) )

if __name__ == "__main__":
	main(RotatingTable)


