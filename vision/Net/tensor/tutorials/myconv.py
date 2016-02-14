import tensorflow as tf

n_classes = 4

x = tf.placeholder(tf.float32, [None, 125, 125, 1])
y = tf.placeholder(tf.float32, [None, n_classes])

def conv2d(img, w, b):
    return tf.nn.relu(tf.nn.bias_add(tf.nn.conv2d(img, w, strides=[1, 1, 1, 1], padding='SAME'),b))

def max_pool(img, k):
    return tf.nn.max_pool(img, ksize=[1, k, k, 1], strides=[1, k, k, 1], padding='SAME')


def conv_net(_X, _weights, _biases, _dropout):
    _X = tf.reshape(_X, shape=[-1, 125, 125, 1])
    conv1 = conv2d(_X, _weights['wc1'], _biases['bc1'])
    #conv1 = max_pool(conv1, k=2)
    
    dense1 = tf.reshape(conv1, [-1, _weights['wd1'].get_shape().as_list()[0]]) # Reshape conv2 output to fit dense layer input
    dense1 = tf.nn.relu(tf.add(tf.matmul(dense1, _weights['wd1']), _biases['bd1'])) # Relu activation
    out = tf.add(tf.matmul(dense1, _weights['out']), _biases['out'])     
    return out



weights = {
    'wc1': tf.Variable(tf.random_normal([5, 5, 1, 32])), # 5x5 conv, 1 channel input, 32 outputs
    'wd1': tf.Variable(tf.random_normal([1, 1024])), # fully connected, 7*7*64 inputs, 1024 outputs
    'out': tf.Variable(tf.random_normal([1024, n_classes])) # 1024 inputs, 10 outputs (class prediction)
}

biases = {
    'bc1': tf.Variable(tf.random_normal([32])),
    'bd1': tf.Variable(tf.random_normal([1024])),
    'out': tf.Variable(tf.random_normal([n_classes]))
}

pred = conv_net(x, weights, biases, None)
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(pred, y))
optimizer = tf.train.AdamOptimizer(learning_rate=.001).minimize(cost)

init = tf.initialize_all_variables()
with tf.Session() as sess:
    sess.run(init)
    step = 1
    # Keep training until reach max iterations
    saver = tf.train.Saver()
    save_path = saver.save(sess, './myconv.ckpt')

    

