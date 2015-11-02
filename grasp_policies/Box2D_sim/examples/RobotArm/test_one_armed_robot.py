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
from Classes.randController import *
from Classes.robotParts import *
from Classes.oneArmedRobot import OneArmedRobot
import sklearn
import math
import numpy as np
import scipy.optimize
import time
import cv2
import IPython
import random

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

        self.radius = 18.0

        # Build the edge of a table by connecting multiple edge lines
        table_edge = self.world.CreateStaticBody(position=self.center_pos_)
        
        # Circle table
        table_edge.CreateEdgeChain(self.makeCircleTableVertices(180))

        # Build the center of the table
        circle = b2FixtureDef(shape=b2CircleShape(radius=0.2))
        table_center = self.world.CreateStaticBody(position=self.center_pos_, fixtures=circle)

        # Build the obstruction objects in the table
        box = b2FixtureDef(shape=b2PolygonShape(box=(0.5,0.5)), density=5, friction=0.2)

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
        fixture = b2FixtureDef(shape=body.fixtures[0].shape, density=5, friction=0.2)

        theta = math.radians(-3.0)
        rotation_matrix = np.asmatrix([[math.cos(theta), -1*math.sin(theta)],
                                       [math.sin(theta), math.cos(theta)   ]])
        new_body_position = rotation_matrix * (np.asmatrix(body.position) - np.asmatrix(self.center_pos_)).T + np.asmatrix(self.center_pos_).T
        new_body_position = np.ravel(new_body_position.T)
        return self.world.CreateDynamicBody(position=new_body_position, fixtures=fixture)

    def rotate_ccw(self, body):
        fixture = b2FixtureDef(shape=body.fixtures[0].shape, density=5, friction=0.2)
        theta = math.radians(3.0)
        rotation_matrix = np.asmatrix([[math.cos(theta), -1*math.sin(theta)],
                                            [math.sin(theta), math.cos(theta)   ]])
        new_body_position = rotation_matrix * (np.asmatrix(body.position) - np.asmatrix(self.center_pos_)).T + np.asmatrix(self.center_pos_).T
        new_body_position = np.ravel(new_body_position.T)
        return self.world.CreateDynamicBody(position=new_body_position, fixtures=fixture)

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
        return [(x,y - self.radius) for x,y in vertices]

    def Keyboard(self, key):
        if key==Keys.K_s:
            self.rotate_cw_all()
        elif key==Keys.K_w:
            self.rotate_ccw_all()

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

        # self.initJointAngles = [0.0,0.0,0.0]
        self.arm = OneArmedRobot(transform)
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

    def getImg(self, state):
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
        self.Print(str(state.shape))

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
        
        # if self.rollout:
        #     inpt = self.getControlFromPolicy()
        #     converted = self.convertInput(inpt)
        #     self.applyInput(converted)
        #     state = self.getState()
        #     self.storeState(state)
        #     self.storePolicyInput(inpt)
        #     # get user input
        #     # save control
        #     # apply control from policy

        # elif self.watch:
        #     inpt = self.getGivenInput()
        #     if self.label:
        #         uInpt = self.getUserInput()
        #         converted = np.array([uInpt[0][0], uInpt[0][1], uInpt[1], uInpt[2]])
        #         if converted.any():
        #             # self.drawLabelingCursor(uInpt[0] + self.gripper.getCenterPosition())
        #             self.drawLabelingCursor(uInpt[0] + self.arm.getEndEffector())
        #         # self.storeInput(uInpt)
        #         self.applyInput(inpt)
        #         state = self.getState()
        #         use = self.learner.askForHelp(state)
        #         if use == -1:
        #             self.displayStateUsed(True)
        #             self.storeInput(uInpt)
        #             self.storeState(state)
        #         else:
        #             self.displayStateUsed(False)
        #             self.givenInputs = np.delete(self.givenInputs, self.i, axis=0)
        #             self.i-=1
        #     else:
        #         self.applyInput(inpt)
        #         state = self.getState()
        #         '''if self.givenStates != None:
        #             used = self.checkIfInUsed(state)
        #             self.displayStateUsed(used)'''
        #         self.storeState(state)

        # else:
        #     userInput = self.getUserInput()
        #     self.applyInput(userInput)
        #     state = self.getState()
        #     self.storeState(state)
        #     self.storeInput(userInput)

        # #print(str(self.getStateObjects()))
        # #state = pygame.surfarray.array3d(self.renderer.surface)
        # #self.getImg(state)

        self.i+=1



'''if __name__=="__main__":
     main(Grasping)'''
     