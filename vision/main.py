# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 14:37:41 2015

@author: David
The controller and gripper code was written by David Gealy.
Integration with the hdf and the convnet was written by Jonathan
"""

from zek.TurnTableControl import * 
from zek.PyControl import * 
from zek.xboxController import *
import numpy as np


from Net.pipeline.bincam import BinaryCamera
from Net import constants
import gopt
import cv2
import argparse
import lfd
import caffe
import os


t = TurnTableControl() # the com number may need to be changed. Default of com7 is used
izzy = PyControl("/dev/cu.usbmodem14111",115200, .04, [0,0,0,0,0],[0,0,0]); #same with this
ap = argparse.ArgumentParser()

#scales = [150.0, 150.0, 150.0, 150.0]
#translations = [0.0, 0.0, 0.0, 0.0]
#drift = 20.0

#DIRECT CONTROL

def directControl(options): # use this to 
    c = XboxController([options.scales[0],155,options.scales[1],155,options.scales[2],options.scales[3]])
    #c = XboxController([100,155,155,155,100]) # this was original scaling

    if options.test:
        lfd.test(options, c, izzy, t)
            
    elif options.deploy:
        lfd.deploy(options, c, izzy, t)
            
    elif options.learn:
        lfd.learn(options, c, izzy, t)


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
options.weights_path = constants.ROOT + 'nets/net3/weights_iter_70.caffemodel'

directControl(options)
izzy.stop()
izzy.ser.close()
t.stop()
t.ser.close()
