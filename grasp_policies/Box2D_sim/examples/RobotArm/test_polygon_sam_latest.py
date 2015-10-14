#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version by Ken Lauer / sirkne at gmail dot com
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

"""
test_polygon_sam.py
Creates polygon objects and places them in a scene

Author: Sam Staszak
"""
from framework import *
from xbox_controller import *
import sklearn
import math
import numpy as np
import scipy.optimize
import time
import cv2
import IPython


class Polygon:
    def __init__(self, vertices=None, transform=b2Transform(),
                 massDensity=0.1, friction=0.5, linearDamping=10, angularDamping=10, center=True,
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


class RobotArm:
    def __init__(self, transform, joint_angles):
        self.transform_ = transform

        self.defaultAngles_ = [0.0, 0.0, 0.0]
        if joint_angles is None: joint_angles = self.defaultAngles_
        self.jointAngles_ = joint_angles
        self.transform_ = b2Transform()
        self.transform_.angle = 0.0
        self.transform_.position = (0, 0)
        
        self.w = 2.0
        self.h = 7.5
        w = self.w
        h = self.h

        # Link One
        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        transform.position = (0, h/2.0)
        self.L1_ = Polygon(vertices, transform)

        # Link Two
        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (0, 7)
        transform.position = (0, 3*h/2.0)
        self.L2_ = Polygon(vertices, transform)

        # Link 3
        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (0, 14)
        transform.position = (0, 5*h/2.0)
        self.L3_ = Polygon(vertices, transform)

        # Link 4
        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (0, 21)
        transform.position = (0, 7*h/2.0)
        self.L4_ = Polygon(vertices, transform)  

        
        self.joint1_ = RobotRevoluteJoint((0, h), self.jointAngles_[0], 
                                 -.5*math.pi, .5*math.pi, 1000, False, True)
        self.joint2_ = RobotRevoluteJoint((0, 2*h), self.jointAngles_[1], 
                                 -.5*math.pi, .5*math.pi, 1000, False, True)
        self.joint3_ = RobotRevoluteJoint((0, 3*h), self.jointAngles_[2], 
                                 -.5*math.pi, .5*math.pi, 1000, False, True)
        self.joints_ = [self.joint1_, self.joint2_, self.joint3_]

        self.gripper_ = RobotGripper(self.transform_, h, w)


    def addToWorld(self, world):
        L1 = self.L1_.addToWorld(world, dynamic=False)
        L2 = self.L2_.addToWorld(world)
        L3 = self.L3_.addToWorld(world)
        L4 = self.L4_.addToWorld(world)
        joint1 = self.joint1_.addToWorld(world, self.L1_, self.L2_)
        joint2 = self.joint2_.addToWorld(world, self.L2_, self.L3_)
        joint3 = self.joint3_.addToWorld(world, self.L3_, self.L4_)

        gripper = self.gripper_.addToWorld(world)

        self.wj_ = world.CreateWeldJoint(
            bodyA=L4,
            bodyB=self.gripper_.getPalm(),
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
        self.joint1_.update()
        self.joint2_.update()
        self.joint3_.update()
        self.gripper_.update()

    def getEndEffector(self):
        angles = self.getCurrentAngles()
        start = np.matrix([[0.0],[0.0],[1.0]])
        theta = self.L1_.getAngle()
        position = self.L1_.getPosition()
        T_w = np.matrix([[math.cos(theta),-math.sin(theta),position[0]],
                         [math.sin(theta),math.cos(theta),self.h],
                         [0,0,1.0]])

        #print(str(angles*180/math.pi))

        endEffector = T_w * self.joint1_.getParameterMatrix(angles[0])*\
            self.joint2_.getParameterMatrix(angles[1])*\
            (self.joint3_.getParameterMatrix(angles[2])+\
                 self.gripper_.getParameterMatrix(angles[2]))* start
        
        return (endEffector[0,0], endEffector[1,0])

    def getInvKin(self, xy): 
        
        if xy[0] < 0.0:
            self.jointAngles_ = [0.0, math.pi/3.0, math.pi/3.0]
        else:
            self.jointAngles_ = [0.0, -math.pi/3.0, -math.pi/3.0]
        

        def distanceToDefault(angles, *args):
            sum = np.sum([math.fabs(math.acos(np.dot(np.array([-math.sin(a_i), math.cos(a_i)]),
                                             np.array([-math.sin(d_i), math.cos(d_i)])))*.00001)
                                      for a_i, d_i in zip(angles, self.jointAngles_)])

            return sum

        def findEndEffector(angles, xy):
            start = np.matrix([[0.0],[0.0],[1.0]])
            theta = self.L1_.getAngle()
            position = self.L1_.getPosition()
            T_w = np.matrix([[math.cos(theta),-math.sin(theta),position[0]],
                            [math.sin(theta),math.cos(theta),self.h],
                            [0,0,1.0]])

            #print(str(angles*180/math.pi))

            endEffector = T_w * self.joint1_.getParameterMatrix(angles[0])*\
                self.joint2_.getParameterMatrix(angles[1])*\
                (self.joint3_.getParameterMatrix(angles[2])+\
                     self.gripper_.getParameterMatrix(angles[2]))* start

            #print(str(endEffector))
            s = np.abs(np.array([endEffector[0,0] - xy[0], endEffector[1,0] - xy[1]]))
            
            #print(str(s))

            return s

        return scipy.optimize.fmin_slsqp(func=distanceToDefault, x0=self.jointAngles_,
                                         f_eqcons=findEndEffector, args=(xy,), iprint=0,
                                         bounds=[(-math.pi,math.pi),(-math.pi,math.pi),
                                                 (-math.pi,math.pi)])


    def changeOrientation(self, angle):
        self.jointAngles_[-1] += angle

    def setTargetAngles(self, target_angles):
        self.jointAngles_ = target_angles
        for i in range(len(target_angles)):
            self.joints_[i].setTargetAngle(target_angles[i])

    def getTargetAngles(self):
        return np.array(self.jointAngles_)

    def getGripper(self):
        return self.gripper_

    def getCurrentAngles(self):
        currAngles = [joint.getAngle() for joint in self.joints_]
        return currAngles

    def ready(self):
        return set(round(self.jointAngles_, 2)) == set(round(self.getCurrentAngles, 2))
    
class RobotGripper:
    def __init__(self, transform, height, width):
        self.transform_ = transform
        self.transform_ = b2Transform()
        self.transform_.angle = transform.angle

        self.h_ = height
        self.w_ = width
        h = self.h_
        w = self.w_
        w_small = w/3.0
        h_small = h/2.5

        self.lengthPalm = h/1.3

        self.transform_.position = (0, 8*h/2.0 + w_small/2.0)
        
        # Gripper Palm
        vertices = vertices=[(0,0),(self.lengthPalm,0),(self.lengthPalm,w_small),(0,w_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (0, 28.4167)
        transform.position = (0, 8*h/2.0 + w_small/2.0)
        self.gripperPalm_ = Polygon(vertices, transform)

        # Gripper Left
        vertices = vertices=[(0,0),(w_small,0),(w_small,h_small),(0,h_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (-3, 27.25)
        self.x = w/2.0 + ((self.lengthPalm-w)/2.0 - w_small) + w_small/2.0
        y = 8*h/2.0 + w_small + h_small/2.0
        transform.position = (-self.x, y)
        print(transform.position)
        self.gripperLeft_ = Polygon(vertices, transform)

        # Gripper Right
        vertices = vertices=[(0,0),(w_small,0),(w_small,h_small),(0,h_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle
        # position = (2.75, 27.25)
        transform.position = (self.x, y)
        self.gripperRight_ = Polygon(vertices, transform)


        self.jointLeft_ = RobotPrismaticJoint(axis=(1,0), lower_translation=0.0, 
                                              upper_translation=self.lengthPalm-3*w_small/2.0,
                                              motor_force=90.0, enable_motor=True)
        self.jointRight_ = RobotPrismaticJoint(axis=(1,0), 
                                               lower_translation=-(self.lengthPalm-3*w_small/2.0),
                                               upper_translation=0.0, motor_force=90.0,
                                               enable_motor=True)

    def addToWorld(self, world):
        self.palm_ = self.gripperPalm_.addToWorld(world)
        self.left_ = self.gripperLeft_.addToWorld(world)
        self.right_ = self.gripperRight_.addToWorld(world)
        jointLeft = self.jointLeft_.addToWorld(world, self.palm_, self.left_)
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
                 gripper_state=0.0):
        self.axis_ = axis
        self.lowerTranslation_ = lower_translation
        self.upperTranslation_ = upper_translation
        self.enableLimit_ = enable_limit
        self.motorForce_ = motor_force
        self.enableMotor_ = enable_motor
        self.roc = roc
        self.controller_= PController()
        
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
        
class BoxFactory:
    def __init__(self):
        self.boxes = []

    def createNewBox(self, h, w, transform, startPos, startAngle, world):
        transform = transform
        transform = b2Transform()
        transform.angle = startAngle
        transform.position = startPos

        vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        box = Polygon(vertices, transform)
        self.boxes.append(box)
        
        box.addToWorld(world)
        body = world.CreateDynamicBody(
            angle=transform.angle,
            position=transform.position,
            linearDamping=0.1,
            angularDamping=0.1
            )
        return body

    def createTarget(self, r, transform, startPos, startAngle, world):
        transform = transform
        transform = b2Transform()
        transform.angle = startAngle
        transform.position = startPos

        self.target = Polygon(vertices=None,transform=transform,radius=r)
        self.target.addToWorld(world)
        body = world.CreateDynamicBody(
            angle=transform.angle,
            position=transform.position,
            linearDamping=0.1,
            angularDamping=0.1
            )
        return body 

    def getTarget(self):
        return self.target

    def getBoxes(self):
        return self.boxes

    def getStateBox(self, box):
        return np.hstack((self.boxes[box].getPosition(), self.boxes[box].getAngle()))

    '''def getStateBoxes(self):
        target = np.array([self.target.getPosition(), self.target.getAngle()])
        objects = np.zeros(len(self.boxes)+1, dtype=object)
        objects[0] = target
        for i in range(len(self.boxes)):
            objects[i+1] = self.getStateBox(i)
        return objects'''

    '''def getBoxesOffTable(self, table):
        outOfBounds = []
        transform = b2Transform()
        eIn1 = b2RayCastInput(p1=table[0], p2=table[1], maxFraction=1)
        eOut1 = b2RayCastOutput()
        
        eIn2 = b2RayCastInput(p1=table[1], p2=table[2], maxFraction=1)
        eOut2 = b2RayCastOutput()

        eIn3 = b2RayCastInput(p1=table[2], p2=table[3], maxFraction=1)
        eOut3 = b2RayCastOutput()
        
        eIn4 = b2RayCastInput(p1=table[3], p2=table[0], maxFraction=1)
        eOut4 = b2RayCastOutput()
        for box in self.boxes:
            hit1 = box.fixtures.RayCast(eOut1, eIn1, transform, 0)
            if hit1 or hit2 or hit3 or hit4:
                outOfBounds.append(box)
            else:'''                
            
    
    def getStateBoxes(self):
        objects = np.hstack((self.target.getPosition(), self.target.getAngle()))
        for i in range(len(self.boxes)):
            objects = np.hstack((objects, self.getStateBox(i)))
        return objects

    def resetBoxes(self, world):
        for box in self.boxes:
            world.DestroyBody(box.getBody())
        world.DestroyBody(self.target.getBody())
        self.boxes = []

class PController:
    def __init__(self):
        self.proportionalGain_ = .5

    def control(self, process_var, set_point):
        return self.proportionalGain_*(set_point-process_var)


class GraspingWorld(Framework):
    name="Robotic Arm Simulation"
    #description=""
    hz=4
    zeta=0.7
    def __init__(self, rollout, learner=None, label=False, watch=False, inpt=None, initState=None):
        super(GraspingWorld, self).__init__(gravity = (0,0))
        
        self.contactPoints = []
        self.outOfBounds = []

        self.rollout = rollout
        self.learner = learner
        self.watch = watch
        self.label = label
        self.color = b2Color(0.9,0.9,0.4)
       
        transform = b2Transform()
        transform.angle = 0.0
        transform.position = (0, 0)

        self.initJointAngles = [0.0,0.0,0.0]
        self.arm = RobotArm(transform, self.initJointAngles)
        self.arm.addToWorld(self.world)
        self.gripper = self.arm.getGripper()

        transform = b2Transform()
        transform.angle = 0.0
        transform.position = (0, 0)
        self.startPos = self.gripper.getCenterPosition()
        #print(str(self.startPos))
        self.end = EndEffector(transform, self.startPos, 0.0)
        
        self.table = [(-20,5),(-9,5),(-9,25),(-20,25)]
        self.i = 0
        self.numBoxes = 5
        
        #self.states = self.getState()

        if self.watch:
            self.givenInputs = inpt
            self.initState = initState
            self.states = np.array([self.initState])
            self.generateBoxes(self.numBoxes)
            if self.label:
                self.xboxController = XboxController(.04)
                inputs = self.getUserInput()
                self.inputs = np.array([inputs[0][0], inputs[0][1],
                                        inputs[1], inputs[2]])
                
        else:
            self.generateBoxes(self.numBoxes)
            self.states = self.getState()
            self.initState = None
            if self.rollout:
                self.policyInputs = self.getControlFromPolicy()
            else:
                self.xboxController = XboxController(.04)
                inputs = self.getUserInput()
                self.inputs = np.array([inputs[0][0], inputs[0][1],
                                        inputs[1], inputs[2]])

    def reset(self):
        '''self.wait = True
        self.arm.setTargetAngles(self.initJointAngles)
        self.gripper.reset()
        self.factory.resetBoxes(self.world)'''

        pygame.joystick.quit()
        self.xboxController = None
        self.world.destructionListener=None
        self.world.contactListener = None
        self.world.renderer=None
        

    def generateBoxes(self, num):
        self.factory = BoxFactory()
        transform = b2Transform()
        transform.angle = 0.0
        transform.position = (0, 0)
        
        h = 2.5
        w = 2

        if self.watch and self.initState != None:
            pos = np.array([self.initState[0], self.initState[1]])
            theta = self.initState[2]
            self.factory.createTarget(1.5,transform,pos,theta,self.world)
            for i in range(3, (num+1)*3, 3):
                pos = np.array([self.initState[i], self.initState[i+1]])
                theta = self.initState[i+2]
                self.factory.createNewBox(h,w,transform,pos,theta,self.world)
            self.boxes = self.factory.getBoxes()
        else:
            pos = np.array([-8, 8.75])
            mean = np.zeros(2)
            cov = np.eye(2) * .1
            pos = pos + np.random.multivariate_normal(mean, cov, 1)
            self.factory.createTarget(1.5,transform,pos[0],0.0,self.world)
            for i in range(num):
                if i % 2 != 0:
                    x = self.table[0][0]+3
                else:
                    x = self.table[1][0]-3
                y = -(i**2/1.15) + self.table[2][1]-3
                if i == 1:
                    y += 1.5
                if i == 3:
                    y -= 2
                theta = 0.0
                pos = np.array([x, y])
                mean = np.zeros(2)
                cov = np.eye(2) * .005
                #cov = np.eye(2) * 0
                pos = pos + np.random.multivariate_normal(mean, cov, 1)

                self.factory.createNewBox(h,w,transform,pos[0],theta,self.world)
            self.boxes = self.factory.getBoxes()
    
    
    def drawTable(self):
        self.renderer.DrawPolygon([self.renderer.to_screen(v) for v in self.table],self.color)
    
    def drawCursor(self, pos):
        self.renderer.DrawPoint(self.renderer.to_screen(pos),2.0,self.color)

    def drawLabelingCursor(self, pos):
        self.renderer.DrawPoint(self.renderer.to_screen(pos),2.0,b2Color(0, 0, 1))

    def setControlInput(self, file):
        self.controlInput = np.load(file)

    def getUserInput(self):
        pygame.event.clear()
        state = self.xboxController.getControllerState()
        if state != None:
            userInput = np.array([state['right_stick'],
                                  state['right_trigger'],
                                  state['left_trigger']])
        else:
            userInput = np.array([np.zeros(2),0.0,0.0])

        return userInput

    def applyInput(self, inpt):
        pos = self.end.move(inpt[0])
        self.drawCursor(pos)
        if self.checkInBounds(pos):
            if inpt[1] == 1.0:
                self.gripper.close()
            if inpt[2] == 1.0:
                self.gripper.open()

            q = self.arm.getInvKin(pos)
            self.arm.setTargetAngles(q)

    def getState(self):

        def getStateObjects():
            objects = self.factory.getStateBoxes()
            return objects

        '''def getStateArm():
            gripperState = self.gripper.getState()
            jointAngles = self.arm.getCurrentAngles()
            return np.array([gripperState, jointAngles], dtype=object)''' 
        def getStateArm():
            gripperState = np.array([self.gripper.getState()])
            jointAngles = self.arm.getCurrentAngles()
            return np.hstack((gripperState, jointAngles))
            
        #state = [getStateObjects(), getStateArm()]
        #state = np.array(state, dtype=object)
        state = np.hstack((getStateObjects(), getStateArm()))
        return state
        

    def checkInBounds(self, pos):
        inBounds = math.hypot(pos[0], pos[1]-7) <= math.hypot(self.startPos[0],
                                                             self.startPos[1]-7)
        return inBounds

    def storeState(self, state):
        self.states = np.vstack((self.states, state))

    def getStates(self):
        return self.states

    def storeInput(self, inpt):
        inpt = np.array([inpt[0][0], inpt[0][1], inpt[1], inpt[2]])
        self.inputs = np.vstack((self.inputs, inpt))

    def getInputs(self):
        if self.watch:
            if self.label:
                return self.consolidateInput()
            else:
                return self.givenInputs
        else:
            if self.rollout:
                return self.policyInputs
            else:
                return self.inputs

    def consolidateInput(self):
        newInput = np.array([])        
        #inp = 'inputsTest.npy'
        #inpF = os.path.join('matrices/', inp)
        #np.save(inpF, self.inputs)
        #f2.write(str(self.inputs))
        #f2.close()

        if not self.inputs[0].any():
            newInput = self.givenInputs[0]
        else:
            inpt = self.inputs[0]
            newInput = np.array([1.5*inpt[0],1.5*inpt[1],inpt[2],inpt[3]])
        #j = 1
        for i in range(1, len(self.inputs)):
            if not self.inputs[i].any():
                newInput = np.vstack((newInput, self.givenInputs[i]))
                #self.states = np.delete(self.states, j, axis=0)
                #j-=1
            else:
                inpt = self.inputs[i]
                inpt = np.array([1.5*inpt[0],1.5*inpt[1],inpt[2],inpt[3]])
                newInput = np.vstack((newInput, inpt))
            #j+=1

        return newInput

    '''def getImg(self, state):
        img = cv2.pyrDown((cv2.pyrDown(state)))
        self.Print(str(self.i))
        #newSurface = pygame.surfarray.make_surface(img)
        #save_path = '/home/staszass/sandbox/grasp_policies/Box2D_sim/examples/Images/'
        #filename = os.path.join(save_path, "image"+str(self.i)+".png")
        #pygame.image.save(newSurface,filename)
        winSize = (32,32)
        blockSize = (16,16)
	blockStride = (8,8)
	cellSize = (8,8)
	nbins = 9
	derivAperture = 1
	winSigma = 4.
	histogramNormType = 0
	L2HysThreshold = 2.0000000000000001e-01
	gammaCorrection = 0
	nlevels = 64
	hog = cv2.HOGDescriptor(winSize,blockSize,blockStride,cellSize,nbins,derivAperture,winSigma,
                                histogramNormType,L2HysThreshold,gammaCorrection,nlevels)

        state = hog.compute(img)
        
        f = open('test.txt', 'w')
        f.write(str(state))
        self.Print(str(state.shape))'''

    def storePolicyInput(self, inpt):
        self.policyInputs = np.vstack((self.policyInputs, inpt))

    def getControlFromPolicy(self):
        if len(self.states.shape) == 1:
            action = self.learner.getAction(self.states)
        else:
            action = self.learner.getAction(self.states[self.i])
        return action

    def getGivenInput(self):
        demoInpt = self.givenInputs[self.i]
        return np.array([np.array([demoInpt[0],demoInpt[1]]), round(demoInpt[2]),
                         round(demoInpt[3])])

    def convertInput(self, inpt):
        converted =  np.array([np.array([inpt[0,0], inpt[0,1]]), round(inpt[0,2]), round(inpt[0,3])])
        return converted

    
    def BeginContact(self, contact):
        worldManifold = contact.worldManifold
        self.contactPoints.append({'bodyA':contact.fixtureA.body, 'bodyB':contact.fixtureB.body})

    def EndContact(self, contact):
        worldManifold = contact.worldManifold
        self.contactPoints.remove({'bodyA':contact.fixtureA.body, 'bodyB':contact.fixtureB.body})

    def isTargetCaged(self):
        target = self.factory.getTarget().getBody()
        [left, right] = self.gripper.getGrippers()
        
        '''for point in self.points:
            if point.'''
        try:
            point1 = {'bodyA':left, 'bodyB':target}
            point2 = {'bodyA':right, 'bodyB':target}
            i1 = self.contactPoints.index(point1)
            i2 = self.contactPoints.index(point2)
            #self.Print("FOUND IT")
            return 1
        except ValueError:
            return 0  

    def checkBoxesOutOfBounds(self):
        table = self.table
        transform = b2Transform()
        eIn1 = b2RayCastInput(p1=table[0], p2=table[1], maxFraction=1)
        eOut1 = b2RayCastOutput()
        
        eIn2 = b2RayCastInput(p1=table[1], p2=table[2], maxFraction=1)
        eOut2 = b2RayCastOutput()

        eIn3 = b2RayCastInput(p1=table[2], p2=table[3], maxFraction=1)
        eOut3 = b2RayCastOutput()
        
        eIn4 = b2RayCastInput(p1=table[3], p2=table[0], maxFraction=1)
        eOut4 = b2RayCastOutput()
        t = self.factory.getTarget()
        target = t.getBody()
        hit1 = target.fixtures[0].RayCast(eOut1, eIn1, 0)
        hit2 = target.fixtures[0].RayCast(eOut2, eIn2, 0)
        hit3 = target.fixtures[0].RayCast(eOut3, eIn3, 0)
        hit4 = target.fixtures[0].RayCast(eOut4, eIn4, 0)
        if self.outOfBounds.count(t) == 0:
            if not self.isTargetCaged():
                if hit1 or hit2 or hit3 or hit4:
                    self.outOfBounds.append(t)
        
        for box in self.boxes:
            if self.outOfBounds.count(box) == 0:
                hit1 = box.getBody().fixtures[0].RayCast(eOut1, eIn1, 0)
                hit2 = box.getBody().fixtures[0].RayCast(eOut2, eIn2, 0)
                hit3 = box.getBody().fixtures[0].RayCast(eOut3, eIn3, 0)
                hit4 = box.getBody().fixtures[0].RayCast(eOut4, eIn4, 0)
                if hit1 or hit2 or hit3 or hit4:
                    self.outOfBounds.append(box)
                else:
                    continue

    def getBoxesOutOfBounds(self):
        return self.outOfBounds

    def displayStateUsed(self, used):
        if used:
            self.Print("THIS STATE WAS USED!")
        else:
            self.Print("THIS STATE WASN'T USED!")

    def checkIfInUsed(self, state):
        for row in self.givenStates:
            if np.array_equal(state, row):
                return True
        return False
        
    def Step(self, settings):
        super(GraspingWorld, self).Step(settings)
        #self.Print("frequency = %g hz, damping ratio = %g" % (self.hz, self.zeta))
        js = self.arm.update()
        self.drawTable()

        #self.isTargetCaged()
        #self.Print(str(len(self.outOfBounds)))
        #self.Print(str(self.outOfBounds))
        self.checkBoxesOutOfBounds()
        #self.Print(str(self.getBoxesOutOfBounds()))

        '''if len(self.points) > 0:
            self.Print(str(self.points[0]['fixtureA'].body))'''

        '''if self.wait:
            print "maybe"
            self.drawCursor(self.gripper.getCenterPosition())

            if self.arm.ready():
                self.startPos = self.gripper.getCenterPosition()
                self.end = EndEffector(transform, self.startPos, 0.0)

                self.generateBoxes(self.numBoxes)

                if self.rollout:
                    self.getControlInput()
                    self.policyInputs = self.getControlFromPolicy()
                else:
                    self.xboxController = XboxController(.04)
                    self.inputs = self.getUserInput()

                    self.states = self.getState()

                self.wait = False
                print "YEAH"'''
        
        if self.rollout:
            inpt = self.getControlFromPolicy()
            converted = self.convertInput(inpt)
            self.applyInput(converted)
            state = self.getState()
            self.storeState(state)
            self.storePolicyInput(inpt)
            # get user input
            # save control
            # apply control from policy

        elif self.watch:
            inpt = self.getGivenInput()
            if self.label:
                uInpt = self.getUserInput()
                converted = np.array([uInpt[0][0], uInpt[0][1], uInpt[1], uInpt[2]])
                if converted.any():
                    # self.drawLabelingCursor(uInpt[0] + self.gripper.getCenterPosition())
                    self.drawLabelingCursor(uInpt[0] + self.arm.getEndEffector())
                # self.storeInput(uInpt)
                self.applyInput(inpt)
                state = self.getState()
                use = self.learner.askForHelp(state)
                if use == -1:
                    self.displayStateUsed(True)
                    self.storeInput(uInpt)
                    self.storeState(state)
                else:
                    self.displayStateUsed(False)
                    self.givenInputs = np.delete(self.givenInputs, self.i, axis=0)
                    self.i-=1
            else:
                self.applyInput(inpt)
                state = self.getState()
                '''if self.givenStates != None:
                    used = self.checkIfInUsed(state)
                    self.displayStateUsed(used)'''
                self.storeState(state)

        else:
            userInput = self.getUserInput()
            self.applyInput(userInput)
            state = self.getState()
            self.storeState(state)
            self.storeInput(userInput)

        #print(str(self.getStateObjects()))
        #state = pygame.surfarray.array3d(self.renderer.surface)
        #self.getImg(state)

        self.i+=1



'''if __name__=="__main__":
     main(Grasping)'''
     
     
