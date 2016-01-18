

class TensorNet():

    def __init__(self):
        raise NotImplementedError

    def save(self):
        raise NotImplementedError
        return self.name

    def load(self):
        raise NotImplementedError
        return self.name

    def train(self):
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
