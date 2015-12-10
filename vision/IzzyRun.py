# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 14:37:41 2015

@author: David
The controller and gripper code was written by David Gealy.
Integration with the hdf and the convnet was written by Jonathan
"""

from TurnTableControl import * 
from PyControl import * 
from xboxController import *
import numpy as np


from pipeline.bincam import BinaryCamera
import constants
import gopt
import cv2
import argparse
import lfd
import caffe
import os


t = TurnTableControl() # the com number may need to be changed. Default of com7 is used
izzy = PyControl("/dev/cu.usbmodem14111",115200, .04, [0,0,0,0,0],[0,0,0]); #same with this
ap = argparse.ArgumentParser()

scales = [150.0, 150.0, 150.0, 150.0]
translations = [0.0, 0.0, 0.0, 0.0]
drift = 20.0

#DIRECT CONTROL

def directControl(options): # use this to 
    c = XboxController([40,155,120,155,90,100])
    #c = XboxController([100,155,155,155,100]) # this was original scaling

if options.test:
        ldf.test(options, c, izzy, t)
            
    elif options.deploy:
        ldf.deploy(options, c, izzy, t)
            
    elif options.learn:
        lfd.learn(options, c, izzy, t)

def revert_controls(controls):
    for i in range(len(controls)):
        controls[i] = (controls[i] - translations[i]) * scales[i] * 1.2
        if abs(controls[i]) < drift:
            controls[i] = 0.0
    return [controls[0], 0.0, controls[1], 0.0, controls[2], controls[3]]

def transform_controls(controls):
    controls = [controls[0], controls[2], controls[4], controls[5]]
    for i in range(len(controls)):
        controls[i] = controls[i] / scales[i] + translations[i]
    return controls
   
def getSimpleState():
    # this returns the state of the 4 axis
    state = izzy.getState()  
    simpleState = [state[0],state[2],state[4]]+t.getState()
    return simpleState




ap.add_argument('-l', '--learn', required=False, action='store_true')
ap.add_argument('-t', '--test', required=False, action='store_true')
ap.add_argument('-d', '--deploy', required=False, action='store_true')
ap.add_argument('-s', '--show', required=False, action='store_true')
args = vars(ap.parse_args())


options = gopt.GripperOptions()
options.test = args['test']
options.deploy = args['deploy']
options.learn = args['learn']
options.show = args['show']

options.model_path = constants.ROOT + 'nets/net3/model3.prototxt'
options.weight_path = constants.ROOT + 'weights_iter_110.caffemodel'

directControl(options)
izzy.stop()
izzy.ser.close()
t.stop()
t.ser.close()
