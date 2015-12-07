# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 14:37:41 2015

@author: David
"""

from TurnTableControl import * 
from PyControl import * 
from xboxController import *
import numpy as np
from pipeline.bincam import BinaryCamera
import constants
import cv2
import argparse
import caffe
import runner


t = TurnTableControl() # the com number may need to be changed. Default of com7 is used
izzy = PyControl("/dev/cu.usbmodem14111",115200, .04, [0,0,0,0,0],[0,0,0]); #same with this
ap = argparse.ArgumentParser()
scales = [40, 120, 90, 100]
drift = 20.0

#DIRECT CONTROL

def directControl(test=False, deploy=False, learn=False): # use this to 
    c = XboxController([40,155,120,155,90,100])
    #c = XboxController([100,155,155,155,100]) # this was original scaling

    bincam = BinaryCamera("./pipeline/meta.txt")
    bincam.open()
    if test:
        while True:
            frame = bincam.read_binary_frame(show=True)
            controls = c.getUpdates()
            print "test: " + str(controls)
            if controls is None:
                print "Done"
                izzy.stop()
                break
            controls[1] = 0
            controls[3] = 0
            izzy.control(controls)
            t.control([controls[5]])
            time.sleep(.05)
            
    if deploy:
        frame = bincam.read_binary_frame()        
        net = caffe.Net(constants.ROOT + 'model.prototxt', constants.ROOT + '', caffe.TEST)
        while True:
            # bincams go from 0 to 255 in 1 dim
            data4D = np.zeros([1, 3, 125, 125])
            frame = frame / 255.0
            data4D[0,0,:,:] = frame
	    data4D[0,1,:,:] = frame
	    data4D[0,2,:,:] = frame
            
            net.forward_all(data=data4D)
            controls = net.blobs['out'].data.copy()[0]
            
            # scale controls and squash small controls
            controls = expand_controls(controls)
                        
            izzy.control(controls)
            t.control([controls[5]])
            time.sleep(.05)
            frame = bincam.read_binary_frame()
            
    if learn:
        i = 0

        # first clear all previous data, then write over
        #open('/Users/JonathanLee/Desktop/Net/hdf/train.txt', 'w').close()
        #writer = open('/Users/JonathanLee/Desktop/Net/hdf/train.txt', 'w+')
        open(constants.ROOT + 'hdf/train-sample.txt', 'w').close()
        writer = open(constants.ROOT + 'hdf/test-sample.txt', 'w+')

        while True:
            controls = c.getUpdates()     
            print controls
            if controls is None:
                print "Done"
                izzy.stop()
                break 
            controls[1] = 0
            controls[3] = 0
            izzy.control(controls)
            t.control([controls[5]])
        
            # store this however you please. (concatenate into array?)
            simpleControls = [controls[0], controls[2], controls[4], controls[5]]
            simpleControls = [ sc/scale for sc, scale in zip(simpleControls, scales) ]
            if not all(int(c)==0 for sc in simpleControls):
                frame = bincam.read_frame(show=False)
                #path = '/Users/JonathanLee/Desktop/sandbox/vision/Net/images_train/img_' + str(i) + '.jpg'
                path = constants.IMAGES_TRAIN_DIR + "img_" + str(i) + ".jpg"
                save_example(writer, path, frame, simpleControls)
                #cv2.imwrite(path, frame)
                #controls_string = ""
                #for sc in simpleControls:
                #    controls_string += ' ' + str(sc)
                #writer.write(path + controls_string + '\n')
                i+=1
             
            #print getSimpleState()
            time.sleep(.05)

def expand_controls(controls):
    for i in range(len(controls)):
        controls[i] = controls[i] * scales[i]
        if controls[i] < drift:
            controls[i] = 0.0
    return [controls[0], 0.0, controls[1], 0.0, controls[2], controls[3]]
    

def save_example(writer, path, frame, controls):
    cv2.imwrite(path, frame)
    controls_string = ""
    for c in controls:
        controls_string += " " + str(c)
    writer.write(path + controls_string + '\n')


def simpleControl(controls): # controls = rotation,extension,grip,turntable (run this in a loop)
    # this converts a list of 4 control inputs into the necessary 6 and sends them off
    izzy.control( [controls[0]]+[0]+[controls[1]]+[0]+[controls[2]]+[0])
    t.control([controls[4]])
    
def getSimpleState():
    # this returns the state of the 4 axis
    state = izzy.getState()  
    simpleState = [state[0],state[2],state[4]]+t.getState()
    return simpleState
        #IF YOU WANT TO AVOID UP AND DOWN, AND WRIST, SEND ZEROS FOR controls[1] and controls[3]


ap.add_argument('-l', '--learn', required=False, action='store_true')
ap.add_argument('-t', '--test', required=False, action='store_true')
ap.add_argument('-d', '--deploy', required=False, action='store_true')
args = vars(ap.parse_args())


#print getSimpleState()
directControl(test=args['test'], learn=args['learn'], deploy=args['deploy'])

# these are needed to close serial connections setup in the begining 
izzy.stop()
izzy.ser.close()
t.stop()
t.ser.close()
