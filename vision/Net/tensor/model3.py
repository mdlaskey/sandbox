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


ROOT = '/Users/JonathanLee/Desktop/sandbox/vision/'

TRAIN_PATH = ROOT + 'Net/hdf/train.txt'
TEST_PATH = ROOT + 'Net/hdf/test.txt'


class NetThree():

    def __init__(self):

        self.x = tf.placeholder('float', shape=[None, 125, 125, 1])
        self.y_ = tf.placeholder("float", shape=[None, 4])

        self.w_conv1 = weight_variable([5, 5, 1, 31])
        self.b_conv1 = bias_variable([32])



    def train(self):


    def save(self):
        

    def load(self):
        
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



