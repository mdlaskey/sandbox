import matplotlib.pyplot as plt
import pandas as pd
from subprocess import call
import argparse
ap = argparse.ArgumentParser()
ap.add_argument('-n', '--net', required=True)
args = vars(ap.parse_args())

if args['net']:    
    train_log = pd.read_csv("./" + args['net'] + "/logs/train.log.train")
    test_log = pd.read_csv("./" + args['net'] + "/logs/train.log.test")
    plt.plot(train_log["NumIters"], train_log["loss"],'b')
    plt.plot(test_log["NumIters"], test_log["loss"], 'r')
    plt.show()
else:
    print "Error: no net specified"
