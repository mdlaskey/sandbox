import argparse
import gopt
from pipeline import bincam
import constants
import cv2
import random


ap = argparse.ArgumentParser()
ap.add_argument('-d', '--dataset', required=True)
ap.add_argument('-s', '--segment', required=False, action='store_true')
args = vars(ap.parse_args())


bc = bincam.BinaryCamera('./pipeline/meta.txt')

ds = args['dataset']
ds_path = "./Net/data/" + ds + "/"

reader = open(ds_path + "controls.txt", 'r')
train_writer = open("Net/hdf/train.txt", 'w')
test_writer = open("Net/hdf/test.txt", 'w')

for line in reader:
    split = line.split(' ')
    filename = split[0]
    controls = [ float(x)/float(s) for x, s in zip(split[1:], gopt.GripperOptions.scales) ]

    if args['segment']:
        im = cv2.imread(ds_path + filename)
        im = bc.pipe(im)
        cv2.imwrite("./net/images/" + filename, im)
    
    controls_string = ""
    for c in controls: controls_string += " " + str(c)
    line = constants.ROOT + 'images/' + filename + controls_string + "\n"
    
    if random.random() > .2:
        train_writer.write(line)
    else:
        test_writer.write(line)


