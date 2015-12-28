import sklearn
import math
import numpy as np
import cv2
import time
from framework import b2Transform
from framework import Keys
from Classes.robotParts import RobotPrismaticJoint
from Classes.robotParts import RobotRevoluteJoint
from Classes.robotParts import RobotGripper
from Classes.robotParts import Polygon

class OneArmedRobot:

	def __init__(self, transform, scale, rev_motor_speed = 100.0, trans_motor_speed = 10.0, gripper_motor_speed = 20.0,
										 rev_max_torque = 10000, trans_max_force = 90.0, gripper_max_force = 180.0):
		self.rev_motor_speed_ = rev_motor_speed
		self.trans_motor_speed_ = trans_motor_speed
		self.gripper_motor_speed_ = gripper_motor_speed
		self.rev_max_torque_ = rev_max_torque
		self.trans_max_force_ = trans_max_force
		self.gripper_max_force_ = gripper_max_force

		# Set up the width and height of polygon resembling the arm
		self.length_ = 4.0
		self.height_ = 4.0

		self.transform_ = transform
		
		# self.transform_ = b2Transform()
		self.transform_.angle = 0.0
		rotation_center = (10.0, 10.0)
		self.transform_.position = rotation_center
	

		# Make an (supposedly) invisible rotational panel as Polygon
		rotating_radius = 2.0

		rotating_panel_transform = b2Transform()
		rotating_panel_transform.angle = self.transform_.angle
		rotating_panel_transform.position = self.transform_.position
		self.rotating_panel_ = Polygon(radius=rotating_radius, transform = rotating_panel_transform)

		# Make a square polygon to held rotating panel in place
		static_length = 2.0
		static_width = 0.5

		vertices = [(0.0,0.0), (static_length,0.0), (static_length, static_width), (0.0, static_width)]
		static_transform = b2Transform()
		static_transform.angle = self.transform_.angle
		static_transform.position = (rotation_center[0], -15.0)
		self.static_panel_ = Polygon(vertices, static_transform)

		# Make moving arm as a polygon
		moving_length = 2.0
		moving_width = 0.5

		vertices = [(0.0,0.0), (moving_length,0.0), (moving_length, moving_width), (0.0, moving_width)]
		moving1_transform = b2Transform()
		moving1_transform.angle = self.transform_.angle
		moving1_transform.position = (rotation_center[0], rotation_center[1] + 2.35)
		self.moving_arm1_ = Polygon(vertices, moving1_transform)

		moving2_transform = b2Transform()
		moving2_transform.angle = self.transform_.angle
		moving2_transform.position = (moving1_transform.position[0], moving1_transform.position[1] + 2 * moving_width )
		self.moving_arm2_ = Polygon(vertices, moving2_transform)


		# Make another stationary arm, attached it to the other end of the moving arm
		extension_length = 1.0
		extension_width = 3.0

		vertices = [(0.0,0.0), (extension_length,0.0), (extension_length, extension_width), (0.0, extension_width)]
		extension_transform = b2Transform()
		extension_transform.angle = self.transform_.angle
		extension_transform.position = (moving1_transform.position[0], moving1_transform.position[1] + moving_width + 7.0)
		self.extension_arm_ = Polygon(vertices, extension_transform)

		# Make robot gripper
		self.transform_.position = (rotation_center[0], rotation_center[1])
		self.gripper_ = RobotGripper(self.transform_, 8.7, 5.5, scale, gripper_motor_speed=self.gripper_motor_speed_, gripper_max_force = self.gripper_max_force_)

		# Make robot prismatic joint
		# axis --> movement in which direction (0,1) means in y-direction(initially)
		self.prismatic_joint_ = RobotPrismaticJoint(axis=(0,1), max_motor_force=self.trans_max_force_, lower_translation = -10.0, upper_translation = 5.0, enable_limit=True)

		# Make the rotational joint
		self.rotational_joint_ = RobotRevoluteJoint(anchor=self.rotating_panel_.transform_.position, limit_enabled=False, max_motor_torque=self.rev_max_torque_)

	def addToWorld(self, world):
		# Add all robot parts to the world of simulation
		# self.test_panel_.addToWorld(world, dynamic=False)
		static_panel = self.static_panel_.addToWorld(world, dynamic = False)
		rotating_panel = self.rotating_panel_.addToWorld(world, dynamic = True)

		moving_arm1 = self.moving_arm1_.addToWorld(world, dynamic = True)
		moving_arm2 = self.moving_arm2_.addToWorld(world, dynamic = True)
		extension_arm = self.extension_arm_.addToWorld(world, dynamic = True)
		gripper = self.gripper_.addToWorld(world)

		edge_anchor = (self.rotating_panel_.transform_.position[0], self.rotating_panel_.transform_.position[1] + self.rotating_panel_.radius)

		# # Connect the rotating panel with the stationary arm using revolute joint
		rotational_joint = self.rotational_joint_.addToWorld(world, moving_arm1, rotating_panel)
		# Connect the extension arm with moving arm using prismatic joint
		self.connector_ = world.CreateWeldJoint(
						bodyA=moving_arm1, bodyB=moving_arm2,
						anchor=moving_arm1.position)

		extension_joint = self.prismatic_joint_.addToWorld(world, extension_arm, moving_arm2)

		self.weld_joint_ = world.CreateWeldJoint(
						bodyA=extension_arm, bodyB=self.gripper_.getPalm(),
						anchor=self.gripper_.getPalm().position
						)


		self.rev_joint_static_ = world.CreateRevoluteJoint(
						bodyA=static_panel, bodyB=rotating_panel,
						anchor=rotating_panel.position
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

	def Keyboard(self, key):
		# Control Robot Arm
		if key == Keys.K_u:
			self.prismatic_joint_.setMotorSpeed(-10.0)
		elif key == Keys.K_j:
			self.prismatic_joint_.setMotorSpeed(10.0)
		elif key == Keys.K_h:
			self.rotational_joint_.increaseMotorSpeed(-100.0)
		elif key == Keys.K_k:
			self.rotational_joint_.increaseMotorSpeed(100)
		elif key == Keys.K_n:
			self.prismatic_joint_.setMotorSpeed(0.0)

		# Control Robot Gripper
		self.gripper_.Keyboard(key)

	############################# Control Method ################################
	def applyControl(self, rotational_control, translation_control, gripper_control):
		self.rotational_joint_.setMotorSpeed(rotational_control * self.rev_motor_speed_)
		self.prismatic_joint_.setMotorSpeed(translation_control * self.trans_motor_speed_)
		self.gripper_.applyControl(gripper_control)

    ############################# Getter Method #################################
    # Important List for what to get :
    # Rotational Joint of Arm:
    #   - motor speed
    #   - angle
    # Translation Joint of Arm:
    #   - motor speed
    # Gripper Finger:
    #   - motor speed 
    # Position:
    #   - Moving Arm
    #   - Gripper Palm
    #   - Gripper Finger
	def getRevoluteAngle(self):
		return self.rotational_joint_.getAngle()

	def getRevoluteMotorSpeed(self):
		return self.rotational_joint_.getMotorSpeed()

	def getTranslationMotorSpeed(self):
		return self.prismatic_joint_.getMotorSpeed()

	def getGripperPalmPosition(self):
		return np.array(self.gripper_.getPalm().position)

	def getGripperFingerPosition(self):
		# A length 2 array consist of position of left finger and right finger respectively
		return self.gripper_.getGrippers()[0].position, self.gripper_.getGrippers()[1].position

	def getGripper(self):
		return self.gripper_
		
	def getGripperState(self):
		return self.gripper_.position, self.gripper_.angle

	def getExtensionArmPosition(self):
		return self.extension_arm_.getPosition()

	def getMovingArm1Position(self):
		return self.moving_arm1_.getPosition()

	def getMovingArm2Position(self):
		return self.moving_arm2_.getPosition()

	def getRobotArmStates(self):
		"""
		col #		State
		0			moving arm 1 position
		1			moving arm 2 position
		2			extension arm position
		3			Gripper's Palm position
		4			Gripper's left finger position
		5			Gripper's right finger position
		6			Angle of Rotational Joint
		"""
		left_pos, right_pos = self.getGripperFingerPosition()
		left_pos = np.array(left_pos)
		right_pos = np.array(right_pos)
		return np.vstack([self.getMovingArm1Position(), 
						  self.getMovingArm2Position(),
						  self.getExtensionArmPosition(),
						  self.getGripperPalmPosition(),
						  left_pos, right_pos])





