import caffe
import hdf
import numpy as np
import os

MODEL = './model.prototxt'
WEIGHTS = './weights_iter_100.caffemodel'

net = caffe.Net(MODEL, WEIGHTS, caffe.TEST)

for filename in os.listdir('./images_test'):
    if filename.endswith('.jpg'):
        image = caffe.io.load_image('./images_test/' + filename)
        #image = hdf.reshape(image)

        data4D = np.zeros([1,3,250,250])
        data4D[0,0,:,:] = image[:,:,0]
	data4D[0,1,:,:] = image[:,:,1]
	data4D[0,2,:,:] = image[:,:,2]

        net.forward_all(data=data4D)
        data = net.blobs['fc2'].data.copy()
        print filename + str([ 150.0 * x for x in data[0] ])
