""" demo script for Robot Arm """
import Box2D
from Box2D.b2 import *
from framework import *
from test_polygon_sam_latest import *
from learner import *
import math
import numpy as np
import os, sys
import time


path = 'matrices/'
inputF = 'inputsDemo'
policyF = 'controlInputsDemo.npy'
policy = os.path.join(path, policyF)

rollout = False
demo = True
example = True

trials = 5
itr = 850

learner = None


if example:
    instance = GraspingWorld
    main(instance)
    ex = instance(False, None, False, None)
    ex.run(10000, "None")
    ex.reset()
    
else:
    for i in range(1,trials+1):
        instance = GraspingWorld
        main(instance)
        inptFile = os.path.join(path, inputF+str(i)+'.npy')
        inpt = np.load(inptFile)
        trial = instance(rollout, learner, demo, inpt)
        message = "Trial " + str(i) + " completed! ESC = Start next trial"
        if i < trials:
            trial.run(itr, message)
        else:
            message = "User trials complete! ESC = Roll out learner policy"
            trial.run(itr, message)
        trial.reset()

    instance = GraspingWorld
    main(instance)
    pInput = np.load(policy)
    trial = instance(rollout, learner, demo, pInput)
    message = "Demo Complete! ESC = Close window"
    trial.run(itr, message)
    trial.reset()



