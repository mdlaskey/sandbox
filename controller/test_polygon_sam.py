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
from xboxController import *
from test_Bridge import create_bridge
import math
import numpy as np
import scipy.optimize
import time
import random

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
        pass

    def getInvKin(self, xy): 
        
        if xy[0] < 0.0:
            self.jointAngles_ = [0.0, math.pi/2.0, math.pi/8.0]
        else:
            self.jointAngles_ = [0.0, -math.pi/2.0, -math.pi/8.0]
        

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

    def getGripper(self):
        return self.gripper_

    def openGripper(self):
        self.gripper_.open()
    def closeGripper(self):
        self.gripper_.close()


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
        x = w/2.0 + ((self.lengthPalm-w)/2.0 - w_small) + w_small/2.0
        y = 8*h/2.0 + w_small + h_small/2.0
        transform.position = (-x, y)
        print(transform.position)
        self.gripperLeft_ = Polygon(vertices, transform)

        # Gripper Right
        vertices = vertices=[(0,0),(w_small,0),(w_small,h_small),(0,h_small)]
        transform = b2Transform()
        transform.angle = self.transform_.angle
        # position = (2.75, 27.25)
        transform.position = (x, y)
        self.gripperRight_ = Polygon(vertices, transform)


        self.jointLeft_ = RobotPrismaticJoint(axis=(1,0), lower_translation=0.0, 
                                           upper_translation=self.lengthPalm-3*w_small/2.0,
                                           motor_force=90.0, enable_motor=True)
        self.jointRight_ = RobotPrismaticJoint(axis=(1,0), lower_translation=-(self.lengthPalm-3*w_small/2.0), 
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

    def open(self):
        self.jointLeft_.open()
        self.jointRight_.open()
    
    def close(self):
        self.jointLeft_.close()
        self.jointRight_.close()

    def getCenterPosition(self):
        return (0, self.left_.position[1])

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
            #print ("current angle: " + str(self.getCurrentAngleDegrees()))
            #print ("motor speed: " + str(motorSpeed))
        else:
            self.setMotorSpeed(0.0)
            #print "DONE"

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

    def open(self):
        if (self.gripperState >= 1.0 - self.roc):
            self.gripperState = 1.0
        else:
            self.gripperState += self.roc

    def close(self):
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
        #print (str(i) + ", " + str(self.pj_.translation))

        if round(self.pj_.translation, 2) != round(i, 2):  
            p_value = self.controller_.control(self.pj_.translation, i)
            self.pj_.motorSpeed = p_value * 1.5
            #print self.pj_.motorSpeed
            #print self.pj_.speed
        else:
            self.pj_.motorSpeed=0.0


class EndEffector:
    def __init__(self, transform, startPos, startAngle):
        self.transform = transform
        
        self.transform = b2Transform()
        self.transform.position = startPos
        self.transform.angle = startAngle

        '''l = .5
        vertices = vertices=[(0,0),(l,0),(l,l),(0,l)]
        self.effector = Polygon(vertices, self.transform)'''
        
    def addToWorld(self, world):
        #effector = self.effector.addToWorld(world)

        self.body = world.CreateDynamicBody(
            angle=self.transform.angle,
            position=self.transform.position,
            linearDamping=0.1,
            angularDamping=0.1
            )
        return self.body

    '''def moveUp(self):
        return self.effector.changePosition((0,.05))

    def moveDown(self):
        return self.effector.changePosition((0,-.05))

    def moveRight(self):
        return self.effector.changePosition((.05,0))

    def moveLeft(self):
        return self.effector.changePosition((-.05,0))'''

    def move(self, state):
        state = state * .04
        state[1] = state[1] * -1
        self.transform.position += state
        return self.transform.position
        
    def rotateC(self):
        return self.effector.changeAngle(-.05)
    
    def rotateCC(self):
        return self.effector.changeAngle(.05)

    def getPosition(self):
        return self.effector.getPosition()


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

    def getBoxes(self):
        return self.boxes        
        
                

class PController:
    def __init__(self):
        self.proportionalGain_ = .5

    def control(self, process_var, set_point):
        return self.proportionalGain_*(set_point-process_var)
        

class PolygonDemo(Framework):
    name="PolygonDemo"
    description="KEYS: j=left l=right i=up k=down f=open d=close"
    hz=4
    zeta=0.7
    def __init__(self):
        super(PolygonDemo, self).__init__(gravity = (0,0))
        pygame.init()
        self.xboxController = XboxController(.04)

        transform = b2Transform()
        transform.angle = 0.0
        transform.position = (0, 0)

        joint_angles = None
        #joint_angles = [0.0, -math.pi/8.0, math.pi/8.0]
        self.arm = RobotArm(transform, joint_angles)
        self.arm.addToWorld(self.world)

        transform = b2Transform()
        transform.angle = 0.0
        transform.position = (0, 0)
        startPos = self.arm.getGripper().getCenterPosition()
        self.end = EndEffector(transform, startPos, 0.0)

        
        self.table = [(-20,5),(-9,5),(-9,25),(-20,25)]

        self.factory = BoxFactory()
        for i in range(6):
            h = random.randrange(2, 4)
            w = random.randrange(2, 4)
            transform = b2Transform()
            transform.angle = 0.0
            transform.position = (0, 0)
            x = random.randrange(self.table[0][0]+4, self.table[1][0]-4)
            y = random.randrange(self.table[0][1]+4, self.table[2][1]-4)
            theta = (-1)**i * math.pi/random.randrange(1, 10)
            self.factory.createNewBox(h,w,transform,(x,y),theta,self.world)
        
        '''[self.pressJ,self.pressI,self.pressK,self.pressL] = [False,False,False,False]
        [self.pressD, self.pressF] = [False,False]
        [self.pressE, self.pressR] = [False,False]

    def Keyboard(self, key):
        if key==Keys.K_j:
            self.pressJ=True
        if key==Keys.K_i:
            self.pressI=True
        if key==Keys.K_k:
            self.pressK=True
        if key==Keys.K_l:
            self.pressL=True
        
        if key==Keys.K_d:
            self.pressD=True
        elif key==Keys.K_f:
            self.pressF=True

        if key==Keys.K_e:
            self.pressE=True
        elif key==Keys.K_r:
            self.pressR=True

    def KeyboardUp(self, key):
        if key==Keys.K_j:
            self.pressJ=False
        if key==Keys.K_i:
            self.pressI=False
        if key==Keys.K_k:
            self.pressK=False
        if key==Keys.K_l:
            self.pressL=False

        if key==Keys.K_d:
            self.pressD=False
        elif key==Keys.K_f:
            self.pressF=False

        if key==Keys.K_e:
            self.pressE=False
        elif key==Keys.K_r:
            self.pressR=False'''

    '''def moveEnd(self, state):
        if self.pressJ:
            return self.end.moveLeft()
        elif self.pressI:
            return self.end.moveUp()
        elif self.pressK:
            return self.end.moveDown()
        elif self.pressL:
            return self.end.moveRight()
        else:
            return self.end.getPosition()

    def rotateEnd(self):
        if self.pressE:
            return self.end.rotateCC()
        if self.pressR:
            return self.end.rotateC()

    def controlGripper(self):
        if self.pressD:
            self.arm.openGripper()
        elif self.pressF:
            self.arm.closeGripper()'''

    def generateBoxes(self, num):
        pass
        

    def Step(self, settings):
        super(PolygonDemo, self).Step(settings)
        #self.Print("frequency = %g hz, damping ratio = %g" % (self.hz, self.zeta))
        js = self.arm.update()

        #self.Print(str(self.arm.getGripper().getCenterPosition()))

        color = b2Color(0.9,0.9,0.4)
        self.renderer.DrawPolygon([self.renderer.to_screen(v) for v in self.table],color)
        
        pygame.event.clear()
        state = self.xboxController.getControllerState()
        if state != None:
            pos = self.end.move(state['right_stick'])
            #self.arm.changeOrientation(.07 * state['left_stick'][0])
            #angle = self.rotateEnd()
            self.renderer.DrawPoint(self.renderer.to_screen(pos),2.0,color)
            if round(state['right_trigger']) == 1.0:
                self.arm.openGripper()
            elif round(state['left_trigger']) == 1.0:
                self.arm.closeGripper()

            q = self.arm.getInvKin(pos)
            #self.Print(str(q*180/math.pi))
            self.arm.setTargetAngles(q)



if __name__=="__main__":
     main(PolygonDemo)
