import argparse
import gopt
from Net.pipeline import bincam
from Net import constants
import cv2
import random


ap = argparse.ArgumentParser()
ap.add_argument('-d', '--datasets', required=True)
ap.add_argument('-s', '--segment', required=False, action='store_true')
args = vars(ap.parse_args())

datasets = args['datasets'].split(' ')


# erase data in files
open("Net/hdf/train.txt", 'w').close()
open("Net/hdf/test.txt", 'w').close()

train_writer = open("Net/hdf/train.txt", "w+")
test_writer = open("Net/hdf/test.txt", "w+")


for dataset in datasets:
    ds_path = "./Net/data/" + dataset + "/"
    reader = open(ds_path + 'controls.txt', 'r')
    bc = bincam.BinaryCamera(ds_path + "meta.txt")

    for line in reader:
        split = line.split(' ')
        filename = split[0]
        new_filename = dataset + "_" + filename
        controls = [ float(x)/float(s) for x, s in zip(split[1:], gopt.GripperOptions.scales) ]

        if args['segment']:
            im = cv2.imread(ds_path + filename)
            im = bc.pipe(im)
            cv2.imwrite("./net/images/" + new_filename, im)
        
        controls_string = ""
        for c in controls: controls_string += " " + str(c)
        line = constants.ROOT + 'images/' + new_filename + controls_string + "\n"
        
        if random.random() > .2:
            train_writer.write(line)
        else:
            test_writer.write(line)


