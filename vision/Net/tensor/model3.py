"""
    Model for net3
        conv
        relu
        fc
        relu
        fc
        tanh


"""


import tensorflow as tf
import inputdata
import random
from tensornet import TensorNet
import time
import datetime


class NetThree(TensorNet):

    batch_size = 100

    ROOT = '/Users/JonathanLee/Desktop/sandbox/vision/'

    TRAIN_PATH = ROOT + 'Net/hdf/train.txt'
    TEST_PATH = ROOT + 'Net/hdf/test.txt'


    def __init__(self):
        self.dir = "./net3/"
        self.name = "net3"

        self.recent = None

        self.x = tf.placeholder('float', shape=[None, 125, 125, 1])
        self.y_ = tf.placeholder("float", shape=[None, 4])

        self.w_conv1 = self.weight_variable([5, 5, 1, 32])
        self.b_conv1 = self.bias_variable([32])

        self.h_conv1 = tf.nn.relu(self.conv2d(self.x, self.w_conv1) + self.b_conv1)

        conv1_num_nodes = NetThree.reduce_shape(self.h_conv1.get_shape())
        fc1_num_nodes = 128
        
        self.w_fc1 = self.weight_variable([conv1_num_nodes, fc1_num_nodes])
        self.b_fc1 = self.bias_variable([fc1_num_nodes])

        self.h_conv1_flat = tf.reshape(self.h_conv1, [-1, conv1_num_nodes])
        self.h_fc1 = tf.nn.relu(tf.matmul(self.h_conv1_flat, self.w_fc1) + self.b_fc1)

        self.w_fc2 = self.weight_variable([fc1_num_nodes, 4])
        self.b_fc2 = self.bias_variable([4])

        self.y_tanh = tf.tanh(tf.matmul(self.h_fc1, self.w_fc2) + self.b_fc2)

        self.loss = tf.reduce_mean(.5*tf.square(self.y_tanh - self.y_))
        self.train_step = tf.train.MomentumOptimizer(.003, .9)
        self.train = self.train_step.minimize(self.loss)

        self.sess = tf.Session()

    def asdf(self):
        print "hi world"

    def optimize(self, iterations, batch_size=100):
        with self.sess.as_default():
            try:
                data = inputdata.InputData(self.TRAIN_PATH, self.TEST_PATH)
                self.sess.run(tf.initialize_all_variables())
                for i in range(iterations):
                    batch = data.next_train_batch(batch_size)
                    ims, labels = batch

                    print("[ Iteration "  + str(i) + "] "),
                    batch_loss = self.loss.eval(feed_dict={self.x: ims, self.y_:labels})
                    print "Training loss: " + str(batch_loss)
                    
                    self.train.run(feed_dict={self.x: ims, self.y_:labels})
                    time.sleep(1.1)

            except KeyboardInterrupt:
                pass
            
            self.save()
            print "Optimize done."
    


    def deploy(self, var_path, sess=None):
        raise NotImplementedError

    def weight_variable(self, shape):
        initial = tf.truncated_normal(shape, stddev=0.1)
        return tf.Variable(initial)

    def bias_variable(self, shape):
        initial = tf.constant(.1, shape=shape)
        return tf.Variable(initial)

    def conv2d(self, x, W):
        return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')
    
    def open(self):
        self.sess = tf.Session()

    def close(self):
        self.sess.close()
    
net = NetThree()
net.load()
