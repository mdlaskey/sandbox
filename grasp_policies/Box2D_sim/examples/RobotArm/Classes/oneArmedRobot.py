import sklearn
import math
import numpy as np
import cv2
import time
import Classes.robotParts.RobotGripper as RobotGripper
import Classes.robotParts.RobotPrismaticJoint as RobotPrismaticJoint
import Classes.robotParts.RobotRevolutejoint as robotRevoluteJoint

class OneArmedRobot:

	def __init__(self, transform):
		self.transform_ = transform
		# self.transform_ = b2Transform()
		self.transform_.angle = 0.0
		self.transform_.position = (0, 0)

		# Set up the width and height of polygon resembling the arm
		self.length_ = 12.0
		self.width_ = 2.0

		# Make moving arm as a polygon
		moving_length = self.length_
		moving_width = self.width_

		vertices = [(0,0), (moving_length,0), (moving_length, moving_width), (0, moving_width)]
		transform = b2Transform()
		transform.angle = self.transform_.angle
		transform.position = (moving_length / 2, moving_width / 2)
		self.moving_arm_ = Polygon(vertices, transform)

		# Make stationary arm as a polygon
		stationary_length = self.length_ - 2.0
		stationary_width = self.width_ + 2.0

		vertices = [(0,0), (stationary_length,0), (stationary_length, stationary_width), (0, stationary_width)]
		transform = b2Transform()
		transform.angle = self.transform_.angle
		transform.position = (stationary_length / 2, stationary_width / 2)
		self.stationary_arm_ = Polygon(vertices, transform)

		# Make robot gripper
		self.gripper_ = RobotGripper(self.transform_, self.length_, self.width_)

		# Make robot prismatic joint
		self.prismatic_joint_ = RobotPrismaticJoint(motor_force=0.5)

	def addToWorld(self, world):
		# Add all robot parts to the world of simulation
		stationary_arm = self.stationary_arm_.addToWorld(world, dynamic=false)
		moving_arm = self.moving_arm_.addToWorld(world)
		gripper = self.gripper_.addToWorld(world)

		# Connect the stationary arm with moving arm using prismatic joint
		prismatic_joint = self.prismatic_joint_.addToWorld(world, stationary_arm, moving_arm)

		self.weld_joint_ = world.CreateWeldJoint(
						bodyA=moving_arm, bodyB=self.gripper_.getPalm(),
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
    	self.gripper_.update()

    def getEndEffector(self):


    def getInvKin(self):

    def getGripper(self):
    	return self.gripper_





