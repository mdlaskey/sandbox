"""
    net3 architecture implemented in tensorflow (additional max pooling has been added for experimentation)

    Layers:
        convolutional 1 stride, 5x5 filter, 32 output channels
        maxpool 2x2
        relu
        fully connected 128 nodes
        fully connected 4 nodes
        tanh

"""

# TODO: remove max pooling layer (reshape dimension of following weights)
# TODO: implement loss visualization (with test loss)


import tensorflow as tf
import numpy as np
import random
import inputdata
import cv2
import time
import datetime

ROOT = '/Users/JonathanLee/Desktop/sandbox/vision/'

TRAIN_PATH = ROOT + 'Net/hdf/train.txt'
TEST_PATH = ROOT + 'Net/hdf/test.txt'

#sess = tf.InteractiveSession()
sess = tf.Session()


def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)

def bias_variable(shape):
    initial = tf.constant(.1, shape=shape)
    return tf.Variable(initial)

def conv2d(x, W):
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

def max_pool_2x2(x):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides = [1, 2, 2, 1], padding='SAME')


#x = tf.placeholder('float', shape=[None, 784])

x = tf.placeholder('float', shape=[None, 125, 125, 1])
y_ = tf.placeholder("float", shape=[None, 4])


W_conv1 = weight_variable([5, 5, 1, 32]) # 5x5 1 input channel, 32 output channels
b_conv1 = bias_variable([32])

h_conv1 = tf.nn.relu(conv2d(x, W_conv1) + b_conv1)
#h_pool1 = max_pool_2x2(h_conv1)


#print h_conv1.get_shape()
#print h_pool1.get_shape()

#W_fc1 = weight_variable([63 * 63 * 32, 128])
W_fc1 = weight_variable([125 * 125 * 32, 128])
b_fc1 = bias_variable([128])

#h_pool1_flat = tf.reshape(h_pool1, [-1, 63 * 63 * 32])
h_pool1_flat = tf.reshape(h_conv1, [-1, 125 * 125 * 32])
h_fc1 = tf.nn.relu(tf.matmul(h_pool1_flat, W_fc1) + b_fc1)

W_fc2 = weight_variable([128, 4])
b_fc2 = bias_variable([4])

y_tanh = tf.tanh(tf.matmul(h_fc1, W_fc2) + b_fc2)


loss = tf.reduce_mean(.5*tf.square(y_tanh - y_))
train_step = tf.train.MomentumOptimizer(.003, .9)
train = train_step.minimize(loss)



data = inputdata.InputData(TRAIN_PATH, TEST_PATH)

""" Loading weight/bias variables and run loss on several batches """
with sess.as_default():
    sess.run(tf.initialize_all_variables())
    saver = tf.train.Saver()
    saver.restore(sess, "net3_01-13-2016_04h44m45s.ckpt")

    for i in range(0, 30):
        batch = data.next_train_batch(100)
        ims, labels = batch
        batch_loss = loss.eval(feed_dict={x: ims, y_:labels})
        print "[Testing: " + str(i) + "] Training loss " + str(batch_loss)


""" Training and saving weight/bias variables
with sess.as_default():
    
    try:
        sess.run(tf.initialize_all_variables())
        for i in range(300):
            batch = data.next_train_batch(100)
            ims, labels = batch
        
            batch_loss = loss.eval(feed_dict={x: ims, y_:labels})
            print "[Iteration: " + str(i) + "] Training loss: " + str(batch_loss)
        
            train.run(feed_dict={x: ims, y_: labels})

            time.sleep(.3)  # to avoid ridiculous memory consumption
    except KeyboardInterrupt:
        pass

    
    print "Saving..."
    saver = tf.train.Saver()
    model_name = "net3_" + datetime.datetime.now().strftime("%m-%d-%Y_%Hh%Mm%Ss") + ".ckpt"
    save_path = saver.save(sess, model_name)
    print "saved to: " + save_path
"""


print "Done."

np.set_printoptions(threshold=np.nan)

"""
images, labels = batch
label = labels[0]
im = np.reshape(images[0], (-1, 125, 125, 1))
#print y_tanh.eval(feed_dict={x: ims})
#print y_tanh.eval(feed_dict={x: im})

for image, label_vec in zip(images, labels):
    image = np.reshape(image, (-1, 125, 125, 1))
    label_vec = np.reshape(label_vec, (-1, 4))
    print loss.eval(feed_dict={x: image, y_:label_vec})

print loss.eval(feed_dict={x: images, y_:labels})
"""

sess.close()
