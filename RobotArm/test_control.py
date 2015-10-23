""" main script for Robot Arm """
from framework import *
from grasping_world import *
from learner import *
import math
import numpy as np
import os, sys
import time

rollout = False
view = False
useSHIV = False
itr = 10000

def run():
    trial = createNewTrainingTrial()
    option = trial.run(itr, "This is done!")
    trial.reset()
     
def createNewTrainingTrial():
    instance = GraspingWorld
    main(instance)
    trial = instance(False, None, False, False, None, None)
    return trial


run()
print "DONE"



