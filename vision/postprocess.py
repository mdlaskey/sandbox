import argparse
import gopt
from Net.pipeline import bincam
import Net.constants
import cv2
import random


ap = argparse.ArgumentParser()
ap.add_argument('-d', '--datasets', required=True)
ap.add_argument('-s', '--segment', required=False, action='store_true')
args = vars(ap.parse_args())
#bc = bincam.BinaryCamera('./Net/pipeline/meta.txt')

datasets = args['datasets'].split(' ')
ds_paths = [ "./Net/data/" + ds + "/" for ds in datasets ]



reader = open(ds_path + "controls.txt", 'r')

# erase data in files
open("Net/hdf/train.txt", 'w').close()
open("Net/hdf/test.txt", 'w').close()

train_writer = open("Net/hdf/train.txt", "w+")
test_writer = opne("Net/hdf/test.txt", "w+")

readers = [ open(ds_path + "controls.txt") for ds_path in ds_paths ]

for reader, ds_path in zip(readers, ds_paths):
    bc = bincam.BinaryCamera(ds_path + "meta.txt")
    for line in reader:
        split = line.split(' ')
        filename = split[0]
        new_filename = ds_path + "_" + filename
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


