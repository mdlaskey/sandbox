import tensorflow as tf
x = tf.placeholder(tf.float32, [None, 125, 125, 1])

W = tf.truncated_normal([5, 5, 1, 32])
b = tf.constant(.1, shape=[32])

conv = tf.nn.conv2d(x, W, strides=[1,1,1,1], padding='SAME')
conv_relu = tf.nn.relu(conv + b)
conv_flat = tf.reshape(conv, [-1, 125*125*32])


W2 = tf.Variable(tf.truncated_normal([125*125*32, 10]))
b2 = tf.Variable(tf.truncated_normal([10]))
y = tf.nn.softmax(tf.matmul(conv_flat, W2) + b2)
y_ = tf.placeholder(tf.float32, [None, 10])
cross_entropy = -tf.reduce_sum(y_*tf.log(y))
train_step = tf.train.GradientDescentOptimizer(0.01).minimize(cross_entropy)

init = tf.initialize_all_variables()
sess = tf.Session()
sess.run(init)

with sess.as_default():
    saver = tf.train.Saver()
    saver.save(sess, './test.ckpt')
    
