import tensorflow as tf
import time
import datetime

class TensorNet():

    def __init__(self):
        raise NotImplementedError

    def save(self):
        print "Saving..."
        saver = tf.train.Saver()
        model_name = self.name + "_" + datetime.datetime.now().strftime("%m-%d-%Y_%Hh%Mm%Ss") + ".ckpt"
        save_path = saver.save(self.sess, self.dir + model_name)
        print "Saved model to " + save_path
        self.recent = save_path
        return

    def load(self, path=None):
        if not path and not self.recent:
            raise Exception("No path to model variables specified")
        
        self.sess = tf.Session()
        with self.sess.as_default():
            self.sess.run(tf.initialize_all_variables())
            saver = tf.train.Saver()
            saver.restore(self.sess, path)


    def optimize(self):
        raise NotImplementedError

    
    @staticmethod
    def reduce_shape(shape):
        """
            Given shape iterable with dimension elements
            reduce shape to total nodes
        """
        shape = [ x.value for x in shape ]
        f = lambda x, y: 1 if y is None else x * y
        return reduce(f, shape, 1)
