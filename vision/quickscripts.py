"""
    module of quick scripts to perform various
    batch processes.
    Don't worry about any of the functions in these, they are just hacks
    but they may be helpful at some point.
"""
import os
import random
import cv2
from pipeline import bincam
import constants

net_path = "/Users/JonathanLee/Desktop/sandbox/vision/Net/"
train_path = net_path + "hdf/train.txt"
test_path = net_path + "hdf/test.txt"
images_train_path = net_path + "images_train/"
images_test_path = net_path + "images_test/"

def rename_dirs():
    """
    Rewrite the paths to images in the train.txt
    file of the Net module to point to appropriate locations
    Store in file trainx.txt to check before permanently changing
    """
    

    train_txt = open(train_path, 'r')
    
    lines = [ x for x in train_txt ]

    train_txt.close()

    train_txt = open(net_path + "hdf/trainx.txt", 'w')
    
    for line in lines:
        split = line.split(' ')
        path = split[0]
        controls = split[1:]
        k = path.rfind('/')
        
        controls_string = ""
        for c in controls:
            controls_string += " " + c

        new_line = images_train_path + path[k+1:] + controls_string
        train_txt.write(new_line)


def export_test():
    """
    randomly export a portion of images in images_train to images_test
    Also renames files in train.txt and test.txt
    Creates new files trainx.txt and testx.txt before permanently moving
    """
    # TODO ^^ this then pipeline, then start training.
    train_txt = open(train_path, 'r')
    lines = [ x for x in train_txt ]
    train_txt.close()

    train_txt = open(net_path + 'hdf/trainx.txt', 'w')
    test_txt = open(net_path + 'hdf/testx.txt', 'w')

    i = 0 
    for line in lines:
        if random.random() > .8:
            i += 1
            new_line = line.replace('images_train', 'images_test')
            path = line.split(' ')[0]
            new_path = new_line.split(' ')[0]
            test_txt.write(new_line)
            os.rename(path, new_path)
        else:
            train_txt.write(line)

    print i
    print len(lines)

def img2bin_train():
    bc = bincam.BinaryCamera('./pipeline/metax.txt')
    bc.close()
    for im_name in os.listdir(images_train_path):
        if im_name.endswith('.jpg'):
            print im_name
            im = cv2.imread(images_train_path + im_name)
            im = bc.pipe(im)
            cv2.imwrite(net_path + 'images_trainx/' + im_name, im)

def img2bin_test():
    """
    convert raw images to binary images
    by feeding through the bincam vision pipeline
    """
    bc = bincam.BinaryCamera('./pipeline/meta.txt')
    bc.close()
    for im_name in os.listdir(images_test_path):
        if im_name.endswith('.jpg'):
            print im_name
            im = cv2.imread(images_test_path + im_name)
            im = bc.pipe(im)
            cv2.imwrite(net_path + 'images_testx/' + im_name, im)


def test_bincam():
    bc = bincam.BinaryCamera('./pipeline/meta.txt')
    bc.open()
    while 1:
        bc.read_binary_frame(show=True)
        key = cv2.waitKey(20)
        if key == 27:
            break


def scale_all_controls():
    train_orig_path = net_path + "hdf/train_orig.txt"
    test_orig_path = net_path + "hdf/test_orig.txt"

    train_path = net_path + "hdf/train.txt"
    test_path = net_path + "hdf/test.txt"

    train_lines = [ x for x in open(train_orig_path, 'r') ]
    test_lines = [ x for x in open(test_orig_path, 'r') ]
    
    train = open(train_path, 'w')
    test = open(test_path, 'w')

    for line in train_lines:
        split = line.split(' ')
        path = split[0]
        controls = [ float(x)/150.0 for x in split[1:] ]
        controls_string = ""
        for c in controls:
             controls_string += " " + str(c)
        new_line = path + controls_string + '\n'
        train.write(new_line)

    for line in test_lines:
        split = line.split(' ')
        path = split[0]
        controls = [ float(x) / 150.0 for x in split[1:] ]
        controls_string = ""
        for c in controls:
             controls_string += " " + str(c)
        new_line = path + controls_string + '\n'
        test.write(new_line)


def scale_controls_smart(scales, translate=0.0):
    """
    read from original train/test path file
    write to trainx and testx file paths
    """
    train_orig_path = net_path + "hdf/old/train_orig.txt"
    test_orig_path = net_path + "hdf/old/test_orig.txt"
    
    trainx_path = net_path + "hdf/trainx.txt"
    testx_path = net_path + "hdf/testx.txt"

    train_lines = [ x for x in open(train_orig_path, 'r') ]
    test_lines = [ x for x in open(test_orig_path, 'r') ]

    trainx = open(trainx_path, 'w')
    testx = open(testx_path, 'w')

    for line in train_lines:
        split = line.split(' ')
        path = split[0]
        
        controls = [float(x)/scale for x, scale in zip(split[1:], scales) ]
        controls_string = ""
        for c in controls:
            controls_string += " " + str(c)
        new_line = path + controls_string + "\n"
        trainx.write(new_line)
    
    for line in test_lines:
        split = line.split(' ')
        path = split[0]

        controls = [float(x)/scale for x, scale in zip(split[1:], scales) ]
        controls_string = ""
        for c in controls:
            controls_string += " " + str(c)
        new_line = path + controls_string + "\n"
        testx.write(new_line)

def translate_controls():
    trainx_path = net_path + "hdf/trainx.txt"
    testx_path = net_path + "hdf/testx.txt"
    
    train_path = net_path + "hdf/train.txt"
    test_path = net_path + "hdf/test.txt"

    train_lines = [ x for x in open(train_path, 'r') ]
    test_lines = [x for x in open(test_path, 'r') ]
    
    trainx = open(trainx_path, 'w')
    testx = open(testx_path, 'w')

    for line in train_lines:
        split = line.split(' ')
        path = split[0]
        
        controls = [float(x)/2.0 + .5 for x in split[1:] ]
        controls_string = ""
        for c in controls:
            controls_string += " " + str(c)
        new_line = path + controls_string + "\n"
        trainx.write(new_line)

    for line in test_lines:
        split = line.split(' ')
        path = split[0]

        controls = [float(x)/2.0 + .5 for x in split[1:] ]
        controls_string = ""
        for c in controls:
            controls_string += " " + str(c)
        new_line = path + controls_string + "\n"
        testx.write(new_line)

def scale_all_images():
    images_train_path = "./Net/images_train/"
    images_test_path = "./Net/images_test/"

    images_train_x = "./Net/images_trainx/"
    images_test_x = "./Net/images_testx/"
    
    for im_name in os.listdir(images_train_path):
        path = images_train_path + im_name
        im = cv2.imread(path)
        im = cv2.resize(im, (125, 125))
        cv2.imwrite(images_train_x + im_name, im)
    for im_name in os.listdir(images_test_path):
        path = images_test_path + im_name
        im = cv2.imread(path)
        im = cv2.resize(im, (125, 125))
        cv2.imwrite(images_test_x + im_name, im)
    
