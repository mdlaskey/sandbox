import h5py
import caffe
import numpy as np
import hdf
import utilities
import argparse

ap = argparse.ArgumentParser()
ap.add_argument('-i', '--images', required=False, action='store_true')
args = vars(ap.parse_args())

hdf.img2hdf('hdf/train')
hdf.img2hdf('hdf/test')

#solver = caffe.get_solver('solver.prototxt')

