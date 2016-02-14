from Net.tensor import net3, inputdata
from options import AMTOptions as opt


train_file = AMTOptions.hdf_dir + "train.txt"
test_file = AMTOptions.hdf_dir + "test.txt"



data = inputdata.AMTData(train_path, test_path)
