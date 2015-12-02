"""
File Contains various classes for establishing a robot. 
Robot Gripper as shown in SHIV paper
Polygon class 
Pristmatic Joint 
Revololute Joint
"""
from framework import *
from test_polygon_sam_latest import *
import sklearn
import math
import numpy as np
import scipy.optimize
import time
import cv2
import IPython


class RobotGripper:
    def __init__(self, transform, height, width, scale):
        self.transform_ = transform
        self.transform_ = b2Transform()
        self.transform_.angle = transform.angle

        self.h_ = height * scale
        self.w_ = width * scale
        h = self.h_
        w = self.w_
        w_small = w * 1.0/3.0
        h_small = h * 2.0/3.0

        # self.h_ = height
        # self.w_ = width
        # h = self.h_
        # w = self.w_
        # # w_small = w/3.0
        # # h_small = h/2.5
        # w_small = 1.9
        # h_small = 


        # self.lengthPalm = 1.53
        # self.heightPalm = 1.67

        # Try to hard code now
        self.lengthPalm_ = 2.5 
        self.heightPalm_ = 2.5

        self.transform_.position = (transform.position[0], 3.9*h/2.0)
        
        # Gripper Palm
        # Make a Base Part
        base_length = 2.5
        base_height = 2.0
        vertices = [(0,0),
                    (0, base_height),
                    (base_length, base_height),
                    (base_length, 0)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        transform.position = (self.transform_.position[0], self.transform_.position[1])
        self.gripperBase_ = Polygon(vertices, transform)

        # Make the additional part
        top_length = 4.1
        top_height = 0.5
        vertices = [(-0.8, 0.0),
                    (-0.8, top_height),
                    (-0.8 + top_length, top_height),
                    (-0.8 + top_length, 0.0)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        transform.position = (self.transform_.position[0], self.transform_.position[1] + 1.0 / 2 * base_height + 1.0/2 * top_height)
        self.gripperPalm_ = Polygon(vertices, transform)

        # Add stationary joint between the base and top
        self.jointBase_ = RobotPrismaticJoint(axis=(1,0), lower_translation=9.0, 
                                              upper_translation=9.0,
                                              motor_force=90.0, enable_motor=True)

        # Gripper Left
        # Base Finger
        finger_base_length = 2.3
        finger_base_height = 1.0
        vertices = [(-finger_base_length,0),
                    (-finger_base_length,finger_base_height),
                    (0, finger_base_height),
                    (0,0)]
        transform = b2Transform()
        transform.angle = self.transform_.angle
        self.x = 5.5 / 2.0
        y = self.transform_.position[1] + 1.0 / 2 * base_height + 1.0/2 * top_height + finger_base_height
        transform.position = (self.transform_.position[0] - self.x, y)
        self.gripperLeft_ = Polygon(vertices, transform)
        # The Finger
        finger_top_length = 1.7
        finger_top_height = 2.1
        vertices = [(-finger_top_length, 0), 
                    (0, finger_top_height),
                    (0, 0)]
        transform = b2Transform()
        transform.angle = self.transform_.angle
        transform.position = (self.transform_.position[0] - self.x + 1.0 / 3 * finger_top_length, y + 2.0 / 3 * finger_top_height)
        self.gripperFingerLeft_ = Polygon(vertices, transform)

        self.jointLeftFinger_ = RobotPrismaticJoint(axis=(1,0), lower_translation=9.0, 
                                                    upper_translation=9.0,
                                                    motor_force=90.0, enable_motor=True)

        # Gripper Right
        # Base
        vertices = [(0,0),
                    (0,finger_base_height),
                    (finger_base_length, finger_base_height),
                    (finger_base_length,0)]
        transform = b2Transform()
        transform.angle = self.transform_.angle
        transform.position = (self.transform_.position[0] + self.x, y)
        self.gripperRight_ = Polygon(vertices, transform)
        # The Finger
        vertices = [(0, 0), 
                    (0, finger_top_height),
                    (finger_top_length, 0)]
        transform = b2Transform()
        transform.angle = self.transform_.angle
        transform.position = (self.transform_.position[0] + self.x - 1.0 / 3 * finger_top_length, y + 2.0 / 3 * finger_top_height)
        self.gripperFingerRight_ = Polygon(vertices, transform)

        self.jointRightFinger_ = RobotPrismaticJoint(axis=(1,0), lower_translation=9.0, 
                                                    upper_translation=9.0,
                                                    motor_force=90.0, enable_motor=True)



        self.jointLeft_ = RobotPrismaticJoint(axis=(1,0), lower_translation=0.0, 
                                              upper_translation=self.lengthPalm_,
                                              motor_force=180.0, enable_motor=True)
        self.jointRight_ = RobotPrismaticJoint(axis=(1,0), 
                                               lower_translation=-(self.lengthPalm_),
                                               upper_translation=0.0, motor_force=180.0,
                                               enable_motor=True)

    def addToWorld(self, world):
        # Bring the Gripper Palm to the simulation
        self.palmBase_ = self.gripperBase_.addToWorld(world)
        self.palm_ = self.gripperPalm_.addToWorld(world)
        # Bring the Left Finger to the simulation
        self.left_ = self.gripperLeft_.addToWorld(world)
        self.leftFinger_ = self.gripperFingerLeft_.addToWorld(world)
        # Brint the Right Finger to the simulation
        self.right_ = self.gripperRight_.addToWorld(world)
        self.rightFinger_ = self.gripperFingerRight_.addToWorld(world)
        jointBase = self.jointBase_.addToWorld(world, self.palmBase_, self.palm_)
        jointLeft = self.jointLeft_.addToWorld(world, self.palm_, self.left_)
        jointLeftFinger_ = self.jointLeftFinger_.addToWorld(world, self.left_, self.leftFinger_)
        jointRightFinger_ = self.jointRightFinger_.addToWorld(world, self.right_, self.rightFinger_)
        jointRight = self.jointRight_.addToWorld(world, self.palm_, self.right_)

        self.body_ = world.CreateDynamicBody(
            angle=self.transform_.angle,
            position=self.transform_.position,
            linearDamping=0.1,
            angularDamping=0.1
            )
        return self.body_

    def getPalm(self):
        return self.palm_

    def getGrippers(self):
        return [self.left_, self.right_]

    def getWorldCenter(self):
        return self.body_.worldCenter

    def update(self):
        self.jointLeft_.update()
        self.jointRight_.update()

    def open(self):
        self.jointLeft_.open()
        self.jointRight_.open()
    
    def close(self):
        self.jointLeft_.close()
        self.jointRight_.close()

    def reset(self):
        self.jointLeft_.resetState()
        self.jointRight_.resetState()

    def getCenterPosition(self):
        return (self.body_.worldCenter[0], self.left_.position[1])

    def getState(self):
        return self.jointLeft_.getState()

    def getParameterMatrix(self, angle):
        t_x=-math.sin(angle)*(self.gripperPalm_.getHeight() + self.gripperLeft_.getHeight()/2.0)
        t_y=math.cos(angle)*(self.gripperPalm_.getHeight() + self.gripperLeft_.getHeight()/2.0)
        parameters = np.matrix([[0, 0, t_x],
                                [0, 0, t_y],
                                [0, 0, 0]]) 

        return parameters

    def Keyboard(self, key):
        if key == Keys.K_y:
            leftVector = self.left_.GetWorldVector(localVector=(500,0))
            rightVector = self.right_.GetWorldVector(localVector=(-500,0))
            self.left_.ApplyForce(leftVector, self.left_.worldCenter, True)
            self.right_.ApplyForce(rightVector, self.right_.worldCenter, True)
        if key == Keys.K_i:
            leftVector = self.left_.GetWorldVector(localVector=(-500,0))
            rightVector = self.right_.GetWorldVector(localVector=(500,0))
            self.left_.ApplyForce(leftVector, self.left_.worldCenter, True)
            self.right_.ApplyForce(rightVector, self.right_.worldCenter, True)


class PController:
    def __init__(self):
        self.proportionalGain_ = .5

    def control(self, process_var, set_point):
        return self.proportionalGain_*(set_point-process_var)

class RobotRevoluteJoint:
    def __init__(self, anchor=(0,0), target_angle=0.0,
                    lower_angle=-math.pi/2.0, upper_angle=math.pi/2.0, max_motor_torque=0.0,
                    limit_enabled=True, motor_enabled=True):
        self.anchor_ = anchor
        self.targetAngle_ = target_angle
        self.lowerAngle_ = lower_angle
        self.upperAngle_ = upper_angle
        self.maxMotorTorque_ = max_motor_torque
        self.limitEnabled_ = limit_enabled
        self.motorEnabled_ = motor_enabled

        self.controller_ = PController()

    def addToWorld(self, world, bodyA, bodyB):
        self.bodyA_ = bodyA
        self.bodyB_ = bodyB
        self.rj_ = world.CreateRevoluteJoint(
            bodyA=bodyA.getBody(),
            bodyB=bodyB.getBody(),
            anchor=self.anchor_,
            lowerAngle=self.lowerAngle_,
            upperAngle=self.upperAngle_,
            motorSpeed=0.0,
            maxMotorTorque=self.maxMotorTorque_,
            enableLimit=self.limitEnabled_,
            enableMotor=self.motorEnabled_
            )
        
        return self.rj_

    def getTargetAngleDegrees(self):
        return self.targetAngle_*180/math.pi
    
    def setTargetAngle(self, new_angle):
        self.targetAngle_= new_angle
        return self.targetAngle_

    def getReferenceAngle(self):
        referenceAngle = self.bodyB_.getAngle() - self.bodyA_.getAngle()
        return referenceAngle

    def getMotorSpeed(self):
        return self.rj_.motorSpeed

    def setMotorSpeed(self, new_speed):
        self.rj_.motorSpeed = new_speed
        return self.rj_.motorSpeed

    def getCurrentAngleDegrees(self):
        return self.rj_.angle*180/math.pi

    def getAngle(self):
        return self.rj_.angle

    def update(self):
        if round(self.getCurrentAngleDegrees(), 2) != round(self.getTargetAngleDegrees(), 2):
            p_value = self.controller_.control(self.rj_.angle, self.targetAngle_)
            motorSpeed = self.setMotorSpeed(p_value*20)
        else:
            self.setMotorSpeed(0.0)

    def getParameterMatrix(self, angle):
        t_x = -math.sin(angle) * self.bodyB_.getHeight()
        t_y = math.cos(angle) * self.bodyB_.getHeight()
        parameters = np.matrix([[math.cos(angle), -math.sin(angle), t_x],
                                [math.sin(angle), math.cos(angle), t_y],
                                [0, 0, 1]])
        return parameters

class RobotPrismaticJoint():
    def __init__(self, axis=(1,0), lower_translation=0.0, upper_translation=0.0,
                 enable_limit=True, motor_force=0.0, enable_motor=True, roc=0.03,
                 gripper_state=0.0, limit=None):
        self.axis_ = axis
        self.lowerTranslation_ = lower_translation
        self.upperTranslation_ = upper_translation
        self.enableLimit_ = enable_limit
        self.motorForce_ = motor_force
        self.enableMotor_ = enable_motor
        self.roc = roc
        self.controller_= PController()
        self.limit_ = limit
        
        self.gripperState = gripper_state

    def addToWorld(self, world, bodyA, bodyB):
        self.pj_ = world.CreatePrismaticJoint(
            bodyA=bodyA,
            bodyB=bodyB,
            anchor=bodyB.worldCenter,
            axis=self.axis_,
            lowerTranslation=self.lowerTranslation_,
            upperTranslation=self.upperTranslation_,
            enableLimit=self.enableLimit_,
            maxMotorForce=self.motorForce_,
            motorSpeed=0.0,
            enableMotor=self.enableMotor_
            )

    def getTranslation(self):
        return self.pj_.translation

    def getSpeed(self):
        return self.pj_.speed

    def getMotorSpeed(self):
        return self.pj_.motorSpeed
    
    def getState(self):
        return self.gripperState

    def resetState(self):
        self.gripperState = 0.0

    def close(self):
        if (self.gripperState >= 1.0 - self.roc):
            self.gripperState = 1.0
        else:
            self.gripperState += self.roc

    def open(self):
        if (self.gripperState <= 0.0 + self.roc):
            self.gripperState = 0.0
        else:
            self.gripperState -= self.roc

    def update(self):
        xp = [0, 1]
        if (self.upperTranslation_ <= 0.0):
            fp = [self.upperTranslation_, self.lowerTranslation_/2.0]
        else:
            fp = [self.lowerTranslation_, self.upperTranslation_/2.0]
        i = np.interp(self.gripperState, xp, fp)

        if round(self.pj_.translation, 2) != round(i, 2):  
            p_value = self.controller_.control(self.pj_.translation, i)
            self.pj_.motorSpeed = p_value * 1.5
        else:
            self.pj_.motorSpeed=0.0



class EndEffector:
    def __init__(self, transform, startPos, startAngle):
        self.transform = transform
        
        self.transform = b2Transform()
        self.transform.position = startPos
        self.transform.angle = startAngle

        
    def addToWorld(self, world):
        self.body = world.CreateDynamicBody(
            angle=self.transform.angle,
            position=self.transform.position,
            linearDamping=0.1,
            angularDamping=0.1
            )
        return self.body

    def move(self, state):
        state = state * .04
        #state[1] = state[1] * -1
        self.transform.position += state
        return self.transform.position
        
    def rotateC(self):
        return self.effector.changeAngle(-.05)
    
    def rotateCC(self):
        return self.effector.changeAngle(.05)

    def getPosition(self):
        return self.transform.position

class Polygon:
    def __init__(self, vertices=None, transform=b2Transform(),
                 massDensity=0.9, friction=0.5, linearDamping=10, angularDamping=10, center=True,
                 radius=None):
        self.transform_ = transform
        self.massDensity_ = massDensity
        self.friction_ = friction
        self.linearDamping_ = linearDamping
        self.angularDamping_ = angularDamping

        #s = b2Separator()

        if vertices == None:
            self.radius = radius
            self.polyShape_ = b2CircleShape(pos=self.transform_.position,
                                            radius=self.radius)
            
        else:
            self.vertices_ = vertices
            vert_array = np.array(self.vertices_)
            self.centroid_ = list(np.mean(vert_array, axis=0))
            if center:
                vert_array_cent = vert_array - self.centroid_
                self.vertices_ = vert_array_cent.tolist()

            self.polyShape_ = b2PolygonShape(vertices=self.vertices_)
        

    @property
    def shape(self):
        return self.polyShape_

    def centerVertices(self, world, dynamic):
        """ Center the polygon vertices at the center of mass for more intuitive pose control """
        localCenter = self.body_.localCenter
#        self.body_.position = self.body_.position - localCenter

        # center the vertex list
        newVertices = []
        for v in self.vertices_:
            newVertices.append((v[0] - localCenter[0], v[1] - localCenter[1]))
        self.vertices_ = newVertices
        self.polyShape_.vertices = self.vertices_
 
        # re-add to the world
        world.DestroyBody(self.body_)
        self.addToWorld(world, center=False, dynamic=dynamic)
       
    def addToWorld(self, world, center = False, dynamic=True):
        """ Add the polygon to the given world as a dynamic body """
        if dynamic:
            self.body_ = world.CreateDynamicBody(
                angle=self.transform_.angle,
                position=self.transform_.position,
                linearDamping=self.linearDamping_,
                angularDamping=self.angularDamping_,
                fixtures=b2FixtureDef(
                    shape=self.polyShape_,
                    density=self.massDensity_,
                    friction=self.friction_
                    )
                )
        else:
            self.body_ = world.CreateStaticBody(
                angle=self.transform_.angle,
                position=self.transform_.position,
                linearDamping=self.linearDamping_,
                angularDamping=self.angularDamping_,
                fixtures=b2FixtureDef(
                    shape=self.polyShape_,
                    density=self.massDensity_,
                    friction=self.friction_
                    )
                )


        if center:
            self.centerVertices(world, dynamic)
        
        return self.body_

    def getHeight(self):
        height = self.vertices_[2][1] - self.vertices_[1][1]
        return height

    def getWidth(self):
        width = self.vertices_[1][0] - self.vertices_[0][0]
        return width

    def getBody(self):
        return self.body_
    
    def setAngle(self, angle):
        self.body_.angle = angle
        return angle
    
    def getAngle(self):
        return self.body_.angle

    def getAngleDegrees(self):
        return self.body_.angle*180/math.pi

    def changePosition(self, position):
        self.body_.position = np.add(self.body_.position, position)
        return self.body_.position

    def changeAngle(self, change):
        self.body_.angle += change
        return self.body_.angle   

    def getPosition(self):
        return np.array([self.body_.position[0], self.body_.position[1]])