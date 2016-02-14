from options import AMTOptions
import tensorflow as tf
from Net.tensor import inputdata, net3, net2

train_file = AMTOptions.hdf_dir + "train.txt"
test_file = AMTOptions.hdf_dir + "test.txt"

data = inputdata.AMTData(train_file, test_file)

net = net3.NetThree()
path = '/Users/JonathanLee/Desktop/sandbox/vision/Net/tensor/./net3/net3_02-13-2016_16h47m09s.ckpt'
net.optimize(100, data=data, path=path, batch_size=50)

