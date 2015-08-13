from framework import *
from test_polygon_sam_latest import *
from learner import *
import math
import numpy as np
import multiprocessing as mp
import os, sys
import time

inf = os.path.join('matrices/', 'inputsDemo4.npy')
inpt = np.load(inf)

itr = 850
in1 = GraspingWorld
main(in1)
in2 = GraspingWorld
main(in2)

ex1 = in1(False, None, True, inpt)
ex2 = in2(False, None, True, inpt)

t1 = mp.Process(target=ex1.run, args=(itr,"none",))
t2 = mp.Process(target=ex2.run, args=(itr,"none",))

t1.start()
t2.start()


print "DONE"

