""" main script for Robot Arm """
from framework import *
import test_one_armed_robot
from Classes.robotParts import *
from Classes.oneArmedRobot import OneArmedRobot
from learner import *
import math
import numpy as np
import os, sys
import time

rollout = False
view = False
useSHIV = True
itr = 900

path = 'matrices/'
pInp = 'pInputsTest.npy'
cInp = 'combinedTest.npy'
r = 'results.npy'
pInpF = os.path.join(path, pInp)
cInpF = os.path.join(path, cInp)
rF = os.path.join(path, r)

results = np.array([])

#np.set_printoptions(suppress=True,precision=3)

learner = Learner()
print results

def rolloutPolicy():
    
    learner.trainModel()
    trial = createRolloutTrial()

    message = "Rollout completed!\n" +\
            "l = label learner policy\n" +\
            "r = replay learner trajectory"
    option = trial.run(itr, message)

    pStates = trial.getStates()
    pInputs = trial.getInputs()

    caged = trial.isTargetCaged()
    outOfBounds = trial.getBoxesOutOfBounds()
    global results
    #print outOfBounds
    #print len(outOfBounds)
    if len(results) == 0:
        results = np.array([caged, len(outOfBounds)])
    else:
        results = np.vstack((results, np.array([caged, len(outOfBounds)])))

    trial.reset()

    if caged and len(outOfBounds) == 0:
        print "Training Successful! Program terminating..."
        return [None, None]

    run = True
    while run:
        if option == "replay":
            trial = createReplayTrial(pInputs, pStates[0])
            option = trial.run(itr, message)
            trial.reset()
        elif option == "label":
            run = False
            return [pInputs,pStates[0]]
        else:
            pass


def labelPolicy(pInputs, initState):
    
    trial = createLabelTrial(pInputs, initState)
    
    run = True
    while run:
        message = "Labeling completed!\n" +\
            "l = relabel learner policy\n" +\
            "r = replay learner trajectory\n" +\
            "s = save and train more\n" +\
            "ESC = end program"
        option = trial.run(itr, message)
        inputs = trial.getInputs()
        states = trial.getStates()
        trial.reset()

        if option == "replay":
            trial = createReplayTrial(pInputs, states)
        elif option == "save":
            learner.updateModel(states, inputs)
            run = False
            return False
        elif option == "label":
            trial = createLabelTrial(pInputs, initState)
        elif option == "end":
            learner.updateModel(states, inputs)
            
            run = False
            return True
        elif option == "continue":
            pass
        else:
            pass

'''def useAHQP(states, inputs):
    statesToUse = np.array([])
    inputsToUse = np.array([])
    for i in range(len(states)):
        use = learner.askForHelp(states[i])
        if use == -1:
            if len(statesToUse) == 0:
                statesToUse = states[i]
                inputsToUse = inputs[i]
            else:
                statesToUse = np.vstack((statesToUse, states[i]))
                inputsToUse = np.vstack((inputsToUse, inputs[i]))
    return [statesToUse, inputsToUse]'''

            

def run():
    learner.initModel(useSHIV)
    end = False
    train()
    while not end:
        [inpts, initState] = rolloutPolicy()
        if inpts == None and initState == None:
            end = True
        else:
            end = labelPolicy(inpts, initState)
    np.save(rF, results)
    learner.saveModel()
    print "status : finish"
        

def train():
    trial = createNewTrainingTrial()
    i=1
    run = True
    while run:
        message = "Trial " + str(i) + " completed!\n" +\
            "e = rollout learner policy\n" +\
            "r = replay your trajectory\n" +\
            "t = try trial again"
        option = trial.run(itr, message)

        states = trial.getStates()
        inputs = trial.getInputs()
        trial.reset()
        
        if option == "retry":
            trial = createNewTrainingTrial()
            
        #elif option == "save":
            #learner.updateModel(states, inputs)
            #trial = createNewTrainingTrial()
            #i+=1
        elif option == "replay":
            trial = createReplayTrial(inputs, states[0])
            
        elif option == "end":
            learner.updateModel(states, inputs)
            run = False
        elif option == "continue":
            run = False
        elif option == "kill":
            run = False
            sys.exit(0)
        else:
            pass
        

    #learner.saveModel()
    
def createNewTrainingTrial():
    instance = test_one_armed_robot.GraspingWorld
    main(instance)
    trial = instance(False, None, False, False, None, None)
    return trial

def createReplayTrial(inpts, state):
    instance = test_one_armed_robot.GraspingWorld
    main(instance)
    print state
    trial = instance(False, None, False, True, inpts, state)
    return trial

def createRolloutTrial():
    rollout = True
    instance = test_one_armed_robot.GraspingWorld
    main(instance)
    trial = instance(rollout, learner, False, False, None, None)
    return trial

def createLabelTrial(inpts, state):
    rollout = False
    instance = test_one_armed_robot.GraspingWorld
    main(instance)
    trial = instance(rollout, learner, True, True, inpts, state)
    return trial


run()
print "DONE"



