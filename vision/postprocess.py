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


