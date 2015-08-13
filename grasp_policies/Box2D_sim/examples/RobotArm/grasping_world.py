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
from system.xbox_controller import *
from system.box_factory import *
from system.end_effector import *
from system.robot_arm import *
from system.gripper import *
from system.polygon import *
import sklearn
import math
import numpy as np
import scipy.optimize
import time
import cv2
        

class GraspingWorld(Framework):
    name="Robotic Arm Simulation"
    #description=""
    hz=4
    zeta=0.7
    def __init__(self, rollout, learner=None, demo=False, inpt=None):
        super(GraspingWorld, self).__init__(gravity = (0,0))

        self.rollout = rollout
        self.demo = demo
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
        self.numBoxes = 5
        self.generateBoxes(self.numBoxes)
        self.i = 0
        
        self.states = self.getState()

        if self.rollout:
            self.learner = learner
            self.policyInputs = self.getControlFromPolicy()
        elif self.demo:
            self.demoInputs = inpt
        else:
            self.xboxController = XboxController(.04)
            inputs = self.getUserInput()
            self.inputs = np.array([inputs[0][0], inputs[0][1],
                                    inputs[1], inputs[2]])
 

        # wait for world to reset
        self.wait = False


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
        self.factory.createTarget(1.5,transform,(-8,8.75),0.0,self.world)
        for i in range(num):
            h = 2.5
            w = 2
            if i % 2 != 0:
                x = self.table[0][0]+5
            else:
                x = self.table[1][0]-3
            y = -(i**2/1.15) + self.table[2][1]-3
            theta=0.0
            self.factory.createNewBox(h,w,transform,(x,y),theta,self.world)
    
    
    def drawTable(self):
        self.renderer.DrawPolygon([self.renderer.to_screen(v) for v in self.table],self.color)
    
    def drawCursor(self, pos):
        self.renderer.DrawPoint(self.renderer.to_screen(pos),2.0,self.color)

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
        return self.inputs

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

    def getPolicyInputs(self):
        return self.policyInputs

    def getControlFromPolicy(self):
        if len(self.states.shape) == 1:
            action = self.learner.getAction(self.states)
        else:
            action = self.learner.getAction(self.states[self.i])
        return action

    def getDemoInput(self):
        demoInpt = self.demoInputs[self.i]
        return np.array([np.array([demoInpt[0],demoInpt[1]]), round(demoInpt[2]),
                         round(demoInpt[3])])

    def convertInput(self, inpt):
        '''if inpt[0,2] > .5:
            inpt[0,2] = 1.0
        else:
            inpt[0,2] = 0.0
        if inpt[0,3] > .3:
            inpt[0,3] = 1.0
        else:
            inpt[0,3] = 0.0'''
        converted =  np.array([np.array([inpt[0,0], inpt[0,1]]), round(inpt[0,2]), round(inpt[0,3])])
        return converted
        
    def Step(self, settings):
        super(GraspingWorld, self).Step(settings)
        #self.Print("frequency = %g hz, damping ratio = %g" % (self.hz, self.zeta))
        js = self.arm.update()
        self.drawTable()

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

        elif self.demo:
            inpt = self.getDemoInput()
            self.applyInput(inpt)

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
     
     
