import matplotlib.pyplot as plt
import pandas as pd
from subprocess import call

call(['/usr/local/caffe/tools/extra/parse_log.py', 'train.log', '.'])

train_log = pd.read_csv("./train.log.train")
test_log = pd.read_csv("./train.log.test")
plt.plot(train_log["NumIters"], train_log["loss"],'r', alpha=0.4)
plt.plot(test_log["NumIters"], test_log["loss"], 'g')
plt.show()
