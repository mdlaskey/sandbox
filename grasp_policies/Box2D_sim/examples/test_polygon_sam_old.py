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
from test_Bridge import create_bridge
import math
import numpy as np
import scipy.optimize
import time

class Polygon:
    def __init__(self, vertices, transform=b2Transform(),
                 massDensity=0.1, friction=0.5, linearDamping=10, angularDamping=10, center=True):
        self.vertices_ = vertices
        self.transform_ = transform
        self.massDensity_ = massDensity
        self.friction_ = friction
        self.linearDamping_ = linearDamping
        self.angularDamping_ = angularDamping

        s = b2Separator()

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
        # vertices = vertices=[(0,0),(w,0),(w,h),(0,h)]
        height = self.vertices_[2][1] - self.vertices_[1][1]
        return height

    def getWidth(self):
        width = self.vertices_[1][0] - self.vertices_[0][0]
        return width

    def getBody(self):
        return self.body_
    
    def getAngle(self):
        return self.body_.angle

    def getAngleDegrees(self):
        return self.body_.angle*180/math.pi

    def getPosition(self):
        return self.body_.position

class RobotArm:
    def __init__(self, transform, joint_angles):
        self.transform_ = transform

        self.defaultAngles_ = [0.0, 0.0, 0.0]
        if joint_angles is None: joint_angles = self.defaultAngles_
        self.jointAngles_ = joint_angles
        self.transform_ = b2Transform()
        self.transform_.angle = 0.0
        self.transform_.position = (0, 0)
        
        w = 3.0
        h = 7.0

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
                                 -.5*math.pi, .5*math.pi, 1000, True, True)
        self.joint2_ = RobotRevoluteJoint((0, 2*h), self.jointAngles_[1], 
                                 -.5*math.pi, .5*math.pi, 1000, True, True)
        self.joint3_ = RobotRevoluteJoint((0, 3*h), self.jointAngles_[2], 
                                 -.5*math.pi, .5*math.pi, 1000, True, True)
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

    def findEndEffector(self):
        palm = self.gripper_.getPalm()
        check = np.matrix([[-math.sin(palm.angle)*(.5+7/4.0) + palm.position[0]],
                           [math.cos(palm.angle)*(.5+7/4.0) + palm.position[1]],
                           [1.0]])

        start = np.matrix([[0.0],[0.0],[1.0]])
        theta = self.L1_.getAngle()
        position = self.L1_.getPosition()
        T_w = np.matrix([[math.cos(theta),-math.sin(theta),position[0]],
                        [math.sin(theta),math.cos(theta),7],
                        [0,0,1.0]])

        endEffector = T_w * self.joint1_.getParameterMatrix()*self.joint2_.getParameterMatrix()*\
            (self.joint3_.getParameterMatrix() + self.gripper_.getParameterMatrix(self.joint3_)) * start

        return [endEffector, check]

    def setTargetAngles(self, target_angles):
        self.jointAngles_ = target_angles
        for i in range(len(target_angle)):
            joint[i].setTargetAngle(target_angles[i])

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
        h_small = h/2.0

        self.transform_.position = (0, 7*h/2.0 + w_small/2.0 + h_small)
        
        # Gripper Palm
        vertices = vertices=[(0,0),(h,0),(h,w_small),(0,w_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (0, 25)
        transform.position = (0, 7*h/2.0 + w_small/2.0 + h_small)
        self.gripperPalm_ = Polygon(vertices, transform)

        # Gripper Left
        vertices = vertices=[(0,0),(w_small,0),(w_small,h_small),(0,h_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle 
        # position = (-3, 27.25)
        x = w/2.0 + ((h-w)/2.0 - w_small) + w_small/2.0
        y = 7*h/2.0 + w_small + 3*h_small/2.0
        transform.position = (-x, y)
        self.gripperLeft_ = Polygon(vertices, transform)

        # Gripper Right
        vertices = vertices=[(0,0),(w_small,0),(w_small,h_small),(0,h_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle
        # position = (2.75, 27.25)
        transform.position = (x, y)
        self.gripperRight_ = Polygon(vertices, transform)


        self.jointLeft_ = RobotPrismaticJoint(axis=(1,0), lower_translation=0.0, 
                                           upper_translation=h-3*w_small/2.0,
                                           motor_force=90.0, enable_motor=True)
        self.jointRight_ = RobotPrismaticJoint(axis=(1,0), lower_translation=-(h-3*w_small/2.0), 
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

    def getWorldCenter(self):
        return self.body_.worldCenter

    def update(self):
        self.jointLeft_.update()
        self.jointRight_.update()

    def getParameterMatrix(self, joint):
        angle = joint.getAngle()
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
        if round(self.getCurrentAngleDegrees(), 4) != round(self.getTargetAngleDegrees(), 4):
            p_value = self.controller_.control(self.rj_.angle, self.targetAngle_)
            motorSpeed = self.setMotorSpeed(p_value*10)
            #print ("current angle: " + str(self.getCurrentAngleDegrees()))
            #print ("motor speed: " + str(motorSpeed))
        else:
            self.setMotorSpeed(0.0)
            #print "DONE"

    def getParameterMatrix(self):
        angle = self.getAngle()
        t_x = -math.sin(angle) * self.bodyB_.getHeight()
        t_y = math.cos(angle) * self.bodyB_.getHeight()
        parameters = np.matrix([[math.cos(angle), -math.sin(angle), t_x],
                                [math.sin(angle), math.cos(angle), t_y],
                                [0, 0, 1]])
        return parameters

class RobotPrismaticJoint():
    def __init__(self, axis=(1,0), lower_translation=0.0, upper_translation=0.0,
                 enable_limit=True, motor_force=0.0, enable_motor=True):
        self.axis_ = axis
        self.lowerTranslation_ = lower_translation
        self.upperTranslation_ = upper_translation
        self.enableLimit_ = enable_limit
        self.motorForce_ = motor_force
        self.enableMotor_ = enable_motor

        self.controller_= PController()

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

    def update(self):
        xp = [0, 1]
        if (self.upperTranslation_ <= 0.0):
            fp = [self.upperTranslation_, self.lowerTranslation_/2.0]
        else:
            fp = [self.lowerTranslation_, self.upperTranslation_/2.0]
        i = np.interp(1, xp, fp)
        #print (str(i) + ", " + str(self.pj_.translation))

        if round(self.pj_.translation, 2) != round(i, 2):  
            p_value = self.controller_.control(self.pj_.translation, i)
            self.pj_.motorSpeed=p_value
            #print self.pj_.motorSpeed
            #print self.pj_.speed
        else:
            self.pj_.motorSpeed=0.0


class PController:
    def __init__(self):
        self.proportionalGain_ = .85

    def control(self, process_var, set_point):
        return self.proportionalGain_*(set_point-process_var)
        

class PolygonDemo(Framework):
    name="PolygonDemo"
    description="Keys: none"
    hz=4
    zeta=0.7
    def __init__(self):
        super(PolygonDemo, self).__init__(gravity = (0,0))

        transform2 = b2Transform()
        #transform2.R = b2Mat22(0, 1, -1, 0)
        transform2.angle = 0.0
        transform2.position = (0, 0)


        joint_angles = [-(math.pi/6.0), (math.pi/3.0), math.pi/6.0]
        self.r_ = RobotArm(transform2, joint_angles)
        self.r_.addToWorld(self.world)

        self.do_ = True


    def Keyboard(self, key):
        pass

    def Step(self, settings):
        super(PolygonDemo, self).Step(settings)
        self.Print("frequency = %g hz, damping ratio = %g" % (self.hz, self.zeta))
        self.r_.update()
        
        #q = self.r_.getInvKin([-5.0, 25])
        #self.Print(q)

        '''self.Print("L4 angle: %g position: %s" % (js[3].getAngle() *180 / math.pi, str(js[3].getPosition()
        self.Print("left translation: %g, speed: %g, motorSpeed: %g" % (js[0].getTranslation(),
                                                                        js[0].getSpeed(),
                                                                        js[0].getMotorSpeed()))
        self.Print("right translation: %g, speed: %g, motorSpeed: %g" % (js[1].getTranslation(),
                                                                        js[1].getSpeed(),
                                                                        js[1].getMotorSpeed()))
        self.Print("joint reference angle: %g" % (js[1].getReferenceAngle()))'''
        end = self.r_.findEndEffector()

        if (self.do_):
            vertices = vertices=[(0,0),(1,0),(1,1),(0,1)]
            transform = b2Transform()
            transform.angle = 0.0 
            transform.position = (end[1][0,0], end[1][1,0])
            self.p_ = Polygon(vertices, transform)
            p = self.p_.addToWorld(self.world)

            vertices1 = vertices=[(0,0),(4,0),(4,4),(0,4)]
            transform = b2Transform()
            transform.angle = 0.0 
            transform.position = (-10, 11)
            self.p1_ = Polygon(vertices1, transform)
            p1 = self.p1_.addToWorld(self.world)

            vertices2 = vertices=[(0,0),(4,0),(4,4),(0,4)]
            transform = b2Transform()
            transform.angle = 0.0 
            transform.position = (10, 20)
            self.p2_ = Polygon(vertices2, transform)
            p2 = self.p2_.addToWorld(self.world)

            vertices3 = vertices=[(0,0),(4,0),(4,4),(0,4)]
            transform = b2Transform()
            transform.angle = 0.0 
            transform.position = (5, 3)
            self.p3_ = Polygon(vertices3, transform)
            p3 = self.p3_.addToWorld(self.world)
            

            vertices4 = vertices=[(0,0),(4,0),(4,4),(0,4)]
            transform = b2Transform()
            transform.angle = 0.0 
            transform.position = (2, 25)
            self.p4_ = Polygon(vertices4, transform)
            p4 = self.p4_.addToWorld(self.world)
            self.do_ = False

        #self.Print(str(self.p_.getPosition()))

        self.Print(str(end[0].round(2)))
        self.Print(str(end[1].round(2)))

if __name__=="__main__":
     main(PolygonDemo)
