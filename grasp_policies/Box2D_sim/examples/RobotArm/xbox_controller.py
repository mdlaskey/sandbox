import pygame
import numpy as np
import sys, os
import IPython 

class XboxController:
    def __init__(self, scale=0.5):
        IPython.embed()
        pygame.joystick.init()
        if(pygame.joystick.get_count() == 0):
            return
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        
        self.lStick = LeftStick(self.controller.get_axis(0),
                                   self.controller.get_axis(1))
        self.rStick = RightStick(self.controller.get_axis(2),
                                     self.controller.get_axis(3))
        #self.dPad = DPad(self.controller.get_hat(0))
        #self.dPad = DPad(self.controller.get_hat())
        # For Mac
        #self.lTrigger = LeftTrigger(self.controller.get_axis(4))
        #self.rTrigger = RightTrigger(self.controller.get_axis(5))

        # For Ubuntu
        self.lTrigger = LeftTrigger(self.controller.get_axis(5))
        self.rTrigger = RightTrigger(self.controller.get_axis(4))

        #self.inUse = [False,False,False,False,False]
        self.inUse = [False,False,False,False]

        
        self.scale = scale
        self.driftLimit = .09
        self.calibrate()

    def calibrate(self):
        self.offset = self.getCurrentState()
        self.uScale = {}
        self.lScale = {}
        for k in self.offset.keys():
            if k == 'left_trigger' or k == 'right_trigger':
                self.uScale[k] = 1
                self.lScale[k] = 1
            else:
                self.uScale[k] = 1/(1-self.offset[k])
                self.lScale[k] = -1/(-1-self.offset[k])

    def getControllerState(self):
        self.update()
        if self.isInUse():
            state = self.getCurrentState()
            for k in state.keys():
                state[k] = state[k] - self.offset[k]
                
                if isinstance(state[k], int) or isinstance(state[k], float):
                    if state[k] > 0.0:
                        state[k] = state[k]*self.uScale[k]
                    else:
                        state[k] = state[k]*self.lScale[k]
                    if abs(state[k]) < self.driftLimit:
                        state[k] = 0.0
                else:
                    for j in range(state[k].size):
                        curr = state[k][j]
                        if curr > 0.0:
                            state[k][j] = curr*self.uScale[k][j]
                        else:
                            state[k][j] = curr*self.lScale[k][j]
                        if abs(state[k][j]) < self.driftLimit:
                            state[k][j] = 0.0
            return state
                
        else:
            return None

    def getCurrentState(self):
        state = {'left_stick':self.lStick.getPos(),
                     'right_stick':self.rStick.getPos(),
                     #'d_pad':self.dPad.getPos(),
                     'left_trigger':self.lTrigger.getPos(),
                     'right_trigger':self.rTrigger.getPos()}
        return state
        
        
    def update(self):
        lstick = self.lStick.setCurrent(self.controller.get_axis(0),
                                   self.controller.get_axis(1))
        rstick = self.rStick.setCurrent(self.controller.get_axis(2),
                                     self.controller.get_axis(3))
        #dpad = self.dPad.setCurrent(self.controller.get_hat(0))
        #dpad = self.dPad.setCurrent(self.controller.get_hat())
        # For Mac
        #ltrigger = self.lTrigger.setCurrent(self.controller.get_axis(4))
        #rtrigger = self.rTrigger.setCurrent(self.controller.get_axis(5))
        # For Ubuntu
        ltrigger = self.lTrigger.setCurrent(self.controller.get_axis(5))
        rtrigger = self.rTrigger.setCurrent(self.controller.get_axis(4))
            

    def isInUse(self):
        '''self.inUse = [self.lStick.isInUse(), self.rStick.isInUse(),
                      self.dPad.isInUse(), self.lTrigger.isInUse(),
                      self.rTrigger.isInUse()]'''

        self.inUse = [self.lStick.isInUse(), self.rStick.isInUse(),
                        self.lTrigger.isInUse(), self.rTrigger.isInUse()]
        for thing in self.inUse:
            if thing:
                return thing

        return False

class LeftStick:
    def __init__(self, axis0, axis1):
        self.initA0 = axis0
        self.initA1 = axis1

        self.a0 = self.initA0
        self.a1 = self.initA1

    def getPos(self):
        return np.array([self.a0, self.a1])

    def setCurrent(self, a0, a1):
        self.a0 = a0
        self.a1 = a1
        return self.getPos()

    def isInUse(self):
        return (self.a0!=self.initA0 or self.a1!=self.initA1)


class RightStick:
    def __init__(self, axis0, axis1):
        self.initA0 = axis0
        self.initA1 = axis1

        self.a0 = self.initA0
        self.a1 = self.initA1

    def getPos(self):
        return np.array([self.a0, -self.a1])

    def setCurrent(self, a0, a1):
        self.a0 = a0
        self.a1 = a1
        return self.getPos()

    def isInUse(self):
        return (self.a0!=self.initA0 or self.a1!=self.initA1)

class DPad:
    def __init__(self, hat):
        self.initH = hat
        self.h = self.initH

    def getPos(self):
        return np.array([self.h[0],self.h[1]])

    def setCurrent(self, h):
        self.h = h
        return self.getPos()

    def isInUse(self):
        return self.h!=self.initH

class LeftTrigger:
    def __init__(self, axis0):
        self.initA0 = axis0
        self.a0 = self.initA0

    def getPos(self):
        if self.a0 <= 0.0:
            return 0.0
        else:
            return 1.0

    def setCurrent(self, a0):
        self.a0 = a0
        return self.getPos()

    def isInUse(self):
        return self.a0!=0.0

class RightTrigger:
    def __init__(self, axis0):
        self.initA0 = axis0
        self.a0 = self.initA0

    def getPos(self):
        if self.a0 <= 0.0:
            return 0.0
        else:
            return 1.0

    def setCurrent(self, a0):
        self.a0 = a0
        return self.getPos()

    def isInUse(self):
        return self.a0!=0.0
