import caffe
import hdf
import numpy as np
import os

MODEL = './nets/net3/model3.prototxt'
WEIGHTS = './weights_iter_110.caffemodel'

net = caffe.Net(MODEL, WEIGHTS, caffe.TEST)

f = open('./hdf/test.txt', 'r')

for line in f:
    path = line.split(' ')[0]
    image = caffe.io.load_image(path)

    data4D = np.zeros([1,3,125,125])
    data4D[0,0,:,:] = image[:,:,0]
    data4D[0,1,:,:] = image[:,:,1]
    data4D[0,2,:,:] = image[:,:,2]

    net.forward_all(data=data4D)
    data = net.blobs['fc2'].data.copy()
    s = ""
    for x in data[0]:
        s += " " + str(x)
    print path + s
