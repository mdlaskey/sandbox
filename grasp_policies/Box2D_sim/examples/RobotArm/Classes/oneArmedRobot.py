import sklearn
import math
import numpy as np
import cv2
import time
from framework import b2Transform
from Classes.robotParts import RobotPrismaticJoint
from Classes.robotParts import RobotRevoluteJoint
from Classes.robotParts import RobotGripper
from Classes.robotParts import Polygon

class OneArmedRobot:

	def __init__(self, transform):
		# Set up the width and height of polygon resembling the arm
		self.length_ = 4.0
		self.height_ = 4.0

		self.transform_ = transform
		# self.transform_ = b2Transform()
		self.transform_.angle = 0.0
		rotation_center = (0.0, 8.0)
		self.transform_.position = rotation_center

		# Make an (supposedly) invisible rotational panel as Polygon
		rotating_radius = 2.0

		rotating_panel_transform = b2Transform()
		rotating_panel_transform.angle = self.transform_.angle
		rotating_panel_transform.position = self.transform_.position
		self.rotating_panel_ = Polygon(radius=rotating_radius, transform = rotating_panel_transform)

		# Make stationary arm as a polygon
		# stationary_length = 1.0
		# stationary_width = 3.0

		# vertices = [(0.0,0.0), (stationary_length,0.0), (stationary_length, stationary_width), (0.0, stationary_width)]
		# stationary_transform = b2Transform()
		# stationary_transform.angle = self.transform_.angle
		# stationary_transform.position = (rotation_center[0], rotation_center[1] + 11.0)
		# self.stationary_arm_ = Polygon(vertices, stationary_transform)

		# Make moving arm as a polygon
		moving_length = 0.5
		moving_width = 7.0

		vertices = [(0.0,0.0), (moving_length,0.0), (moving_length, moving_width), (0.0, moving_width)]
		moving_transform = b2Transform()
		moving_transform.angle = self.transform_.angle
		moving_transform.position = (rotation_center[0] + 1.0, rotation_center[1] + 11.0)
		# moving_transform.position = (stationary_transform.position[0] + stationary_length, stationary_transform.position[1] + stationary_width / 2.0 + 2.0)
		self.moving_arm_ = Polygon(vertices, moving_transform)

		# Make another stationary arm, attached it to the other end of the moving arm
		extension_length = 1.0
		extension_width = 6.0

		vertices = [(0.0,0.0), (extension_length,0.0), (extension_length, extension_width), (0.0, extension_width)]
		extension_transform = b2Transform()
		extension_transform.angle = self.transform_.angle
		extension_transform.position = (moving_transform.position[0] - extension_length, moving_transform.position[1] + moving_width/2.0 + 4.0)
		self.extension_arm_ = Polygon(vertices, extension_transform)

		# Make robot gripper
		self.gripper_ = RobotGripper(self.transform_, 7.5, 1.5)

		# Make robot prismatic joint
		# axis --> movement in which direction (0,1) means in y-direction(initially)
		self.prismatic_joint_ = RobotPrismaticJoint(axis=(0,1), motor_force=0.0, upper_translation = 5.0)

		# Make the rotational joint
		self.rotational_joint_ = RobotRevoluteJoint(anchor= 2 * self.rotating_panel_.transform_.position ,limit_enabled=False)

	def addToWorld(self, world):
		# Add all robot parts to the world of simulation
		rotating_panel = self.rotating_panel_.addToWorld(world, dynamic = False)
		# stationary_arm = self.stationary_arm_.addToWorld(world, dynamic = True)
		moving_arm = self.moving_arm_.addToWorld(world, dynamic = True)
		extension_arm = self.extension_arm_.addToWorld(world, dynamic = True)
		gripper = self.gripper_.addToWorld(world)

		# Connect the rotating panel with the stationary arm using revolute joint
		rotational_joint = self.rotational_joint_.addToWorld(world, self.rotating_panel_, self.moving_arm_)

		# Connect the stationary arm with moving arm using prismatic joint
		# prismatic_joint = self.prismatic_joint_.addToWorld(world, moving_arm, stationary_arm)

		# Connect the extension arm with moving arm using prismatic joint
		extension_joint = self.prismatic_joint_.addToWorld(world, extension_arm, moving_arm)

		self.weld_joint_ = world.CreateWeldJoint(
						bodyA=extension_arm, bodyB=self.gripper_.getPalm(),
						anchor=self.gripper_.getPalm().position
						)

		self.body_ = world.CreateDynamicBody(
                angle=self.transform_.angle,
                position=self.transform_.position,
                linearDamping=0.1,
                angularDamping=0.1
                )
		return self.body_

	def update(self):
		self.prismatic_joint_.update()
		self.rotational_joint_.update()
		self.gripper_.update()

    ############################# Getter Method #################################
	def getCurrentAngles(self):
		return self.rotational_joint_.getAngle()

	def getGripper(self):
		return self.gripper_





