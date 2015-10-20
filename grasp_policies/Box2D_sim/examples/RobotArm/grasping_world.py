"""
grasping_world.py
Main class for grasping example. Handles creation of entire system (arm, gripper, world, etc.).
Stores xbox controller input and state of system. Has three main modes of operation
determined by booleans rollout, label, and watch.

If rollout is True then the world created is for rolling out a policy. Therefore, no user input
is collected and a learner is also passed in, which is used to get the action at a specific state.

If label is True then the world created is for labeling the rolled out policy. It must also
be accompanied by an instance of the learner, watch should be True as well, the inpt will be
the actions from the previous rolled out policy, and the initState is the initial state of the
previous rolled out policy iteration.

Xbox input needs to be converted before it is stored as an input

Fields:

Author: Sam Staszak
"""
from framework import *
from system import *
import pygame
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
    def __init__(self, rollout, learner=None, label=False, watch=False, inpt=None, initState=None):
        super(GraspingWorld, self).__init__(gravity = (0,0))
        
        # For determining if the target is caged, keep list of contact points in world
        self.contactPoints = []
        # For determining if any objects are out of bounds, updated by checkBoxesOutOfBounds
        self.outOfBounds = []

        self.rollout = rollout
        self.learner = learner
        self.watch = watch
        self.label = label

        # Color of end effector dot
        self.color = b2Color(0.9,0.9,0.4)
       
        # Create robot arm
        transform = b2Transform()
        transform.angle = 0.0
        transform.position = (0, 0)
        self.initJointAngles = [0.0,0.0,0.0]
        self.arm = RobotArm(transform, self.initJointAngles)
        self.arm.addToWorld(self.world)
        self.gripper = self.arm.getGripper()

        # Create end effector
        transform = b2Transform()
        transform.angle = 0.0
        transform.position = (0, 0)
        self.startPos = self.gripper.getCenterPosition()
        self.end = EndEffector(transform, self.startPos, 0.0)
        
        # Create bounds of table
        self.table = [(-20,5),(-9,5),(-9,25),(-20,25)]
        self.numBoxes = 5

        # self.i keeps track of the current step of the world
        self.i = 0

        # World is being watched (replay or labeling)
        if self.watch:
            # Inputs already determined
            self.givenInputs = inpt
            self.initState = initState
            self.states = np.array([self.initState])
            self.generateBoxes(self.numBoxes)
            # World being used for labeling
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
        """
	World must be reset after every trial or else things get weird
	"""
        pygame.joystick.quit()
        self.xboxController = None
        self.world.destructionListener=None
        self.world.contactListener = None
        self.world.renderer=None
        

    def generateBoxes(self, num):
        """
	Generates the given number (num) of boxes on the table.
        If the world is being watched or labeled then the initial
        states of the boxes are already determined.
        If not then the target and boxes are given specific positions
	"""
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
        """
	Draws the table
	"""
        self.renderer.DrawPolygon([self.renderer.to_screen(v) for v in self.table],self.color)
    
    def drawCursor(self, pos):
        """
        Draws the end effector
	"""
        self.renderer.DrawPoint(self.renderer.to_screen(pos),2.0,self.color)

    def drawLabelingCursor(self, pos):
        """
	Draws the labeling point
	"""
        self.renderer.DrawPoint(self.renderer.to_screen(pos),2.0,b2Color(0, 0, 1))

    def getUserInput(self):
        """
	Gets the user input from the xbox controller
	"""
        pygame.event.clear()
        state = self.xboxController.getControllerState()
        # If the state is none then the user didn't give input
        # state is returned as a dictionary that can be indexed
        # by specified keys
        if state != None:
            userInput = np.array([state['right_stick'],
                                  state['right_trigger'],
                                  state['left_trigger']])
        else:
            userInput = np.array([np.zeros(2),0.0,0.0])

        return userInput

    def applyInput(self, inpt):
        """
	Applies the given inpt to the world
	"""
        # Moves the end effector and draws it in new position
        pos = self.end.move(inpt[0])
        self.drawCursor(pos)

        # Only moves the arm if the pos is reachable
        if self.checkInBounds(pos):
            if inpt[1] == 1.0:
                self.gripper.close()
            if inpt[2] == 1.0:
                self.gripper.open()

            q = self.arm.getInvKin(pos)
            self.arm.setTargetAngles(q)

    def getState(self):
        """
	Gets the current state of the world
	"""

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
        j = 1
        for i in range(1, len(self.inputs)):
            if not self.inputs[i].any():
                #newInput = np.vstack((newInput, self.givenInputs[i]))
                self.states = np.delete(self.states, j, axis=0)
                j-=1
            else:
                inpt = self.inputs[i]
                inpt = np.array([1.5*inpt[0],1.5*inpt[1],inpt[2],inpt[3]])
                newInput = np.vstack((newInput, inpt))
            j+=1

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
                    self.drawLabelingCursor(uInpt[0] + self.end.getPosition())
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
