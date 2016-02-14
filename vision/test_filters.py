from Net.tensor import net3, inputdata
import numpy as np
from options import AMTOptions as opt
import cv2

def show(im):
    while True:
        cv2.imshow('preview', im)
        if cv2.waitKey(20) == 27:
            break


test_image_path = opt.data_dir + 'color_images/dataset1_img_0.jpg'
test_image = cv2.imread(test_image_path)

model_path = opt.tf_dir + 'net3/net3_02-13-2016_22h52m04s.ckpt'
net = net3.NetThree()
sess = net.load(var_path=model_path)

im = inputdata.im2tensor(test_image, channels=3)
shape = np.shape(im)
im = np.reshape(im, (-1, shape[0], shape[1], shape[2]))



with sess.as_default():
    filter = sess.run(net.h_conv1, feed_dict={net.x:im})
    print filter.shape
    for slice in range(filter.shape[-1]):
        filter_image = filter[0,:,:,slice]
        print filter_image
        cv2.imwrite("filter" + str(slice) + '.jpg', filter_image)