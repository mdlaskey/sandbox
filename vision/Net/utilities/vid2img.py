import cv2
import random
import numpy as np
from constants import IMAGES_TRAIN_DIR, IMAGES_TEST_DIR, IMAGES_TRAIN_TXT, IMAGES_TEST_TXT

# TMP: path to sample video; should change to camera stream
vid_path = '/Users/JonathanLee/Desktop/sample.mov'

def vid2img(prefix=''):
    """
    Split video into frames and save images to trian and test folders.
    Resize images to network specification.
    Write to train and test files to indicate paths to newly created images.
    Images placed in img_dir with optional prefix, if none given, 'img'
    """
    if len (prefix) == 0:
        prefix = 'img'
    cap = cv2.VideoCapture(vid_path) 
    
    # for every frame, path and labels should be written to lists
    train_writer = open(IMAGES_TRAIN_TXT, 'w+')
    test_writer = open(IMAGES_TEST_TXT, 'w+')

    i = 0
    rval, frame = cap.read()
    while frame is not None:
        
        # determine whether train or test
        if random.random() < .8:
            img_path = IMAGES_TRAIN_DIR + '/' + prefix + '_' + str(i) + '.jpg'
            writer = train_writer
        else:
            img_path = IMAGES_TEST_DIR + '/' + prefix + '_' + str(i) + '.jpg'
            writer = test_writer

        # for now randomly defined labels
        controls =[ random.random(), random.random(), random.random(), random.random() ]
        write_path_controls(img_path, controls, writer)
        cv2.imwrite(img_path, cv2.resize(frame, (125 , 125)))
        i += 1
        rval, frame = cap.read()



def write_path_controls(path, controls, writer):
    """
    Write input image path and label controls with writer to file.
    Image should already exist at path.

    "path/to/image.jpg 0 3 1 -2"
    
    path        path to input image
    controls    list of controls to be written
    writer      from open(filename, 'w+')
    """
    controls_string = ""
    for c in controls:
        controls_string += ' ' + str(c)
    writer.write(path + controls_string + '\n')
