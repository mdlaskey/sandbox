import math
import random
import numpy as np
import IPython
from numpy import linalg as LA
from sklearn.kernel_ridge import KernelRidge
from sklearn import preprocessing  
from sklearn import linear_model
from sklearn import metrics 
from scipy.sparse import csr_matrix
from scipy.sparse import vstack
from time import gmtime, strftime
import sys
import os
from AHQP import *


class Learner():

    path = 'matrices/'
    inputF = 'inputs.npy'
    stateF = 'states.npy'
    itrF = 'itr.npy'
    inptFile = os.path.join(path, inputF)
    stateFile = os.path.join(path, stateF)
    itrFile = os.path.join(path, itrF)

    itr = np.array([])

    useSHIV = False
    THRESH = 0.45
    ahqp_solver_g = AHQP()
    ahqp_solver_b = AHQP(sigma=1e-1)


    def trainModel(self, s=None, a=None):
        """
        Trains model on given states and actions.
        Uses neural net or SVM based on global
        settings.
        """
        states, actions = self.states[3:], self.actions[3:]
        #print "states.shape"
        #print states.shape
        #print "actions.shape"
        #print actions.shape

        if len(self.itr) == 0:
            self.itr = np.array([states.shape[0]])
        else:
            self.itr = np.hstack((self.itr, states.shape[0]))

        '''if states.shape[0] > 2700.0:
            f = os.path.join(self.path, 'statesToValidate.npy')
            np.save(f, states)
            IPython.embed()'''

        #actions = actions.ravel()
        self.clf = KernelRidge(alpha=1.0)
        self.clf.kernel = 'rbf'
        print "SIZE: ", states.shape
        self.clf.fit(states, actions)
        actions_pred = self.clf.predict(states)
        bad_state = np.zeros(actions_pred.shape[0])
        for i in range(actions_pred.shape[0]):
            #print LA.norm(actions_pred[i,:] - actions[i,:])
            if(LA.norm(actions_pred[i,:] - actions[i,:])>self.THRESH):
                bad_state[i] = 1

        if self.useSHIV:
            self.labels = np.zeros(states.shape[0])+1.0
            self.scaler = preprocessing.StandardScaler().fit(states)
            states_proc = self.scaler.transform(states)
            
            good_labels = bad_state == 0.0         
            states_g = states_proc[good_labels,:] 

            bad_labels = bad_state == 1.0 
            states_b = states_proc[bad_labels,:] 
            #IPython.embed()
            self.ahqp_solver_g.assembleKernel(states_g, np.zeros(states_g.shape[0])+1.0)
            self.ahqp_solver_b.assembleKernel(states_b, np.zeros(states_b.shape[0])+1.0)
            #IPython.embed()
            self.ahqp_solver_g.solveQP()
            self.ahqp_solver_b.solveQP()

            #score = self.clf.score(states, actions)
            #print score

    def askForHelp(self,state):
        if self.useSHIV:
            state = self.scaler.transform(state)
            if self.ahqp_solver_b.predict(state)==1.0:
                return -1.0
            else:
                return self.ahqp_solver_g.predict(state)
        else:
            return -1


    def getAction(self, state):
	"""
	Returns a prediction given the input state.
	Uses neural net or SVM based on global
	settings.
	"""

	return self.clf.predict(state)


    def initModel(self, useSHIV):
        self.useSHIV = useSHIV
        try:
            self.states = np.load(self.stateFile)
            self.actions = np.load(self.inptFile)
        except IOError:
            self.states = np.array([-8,8.75,0,-12,22,0,-15,21.13043404,
                                     0,-12,18.52173996,0,-15,14.173913,
                                     0,-12,8.08695698,0,0,0,0,0])
            self.actions = np.array([0,0,0,0])
        #self.trainModel(self.states, self.actions)

    def updateModel(self, s, a):
	self.states = np.vstack((self.states, s))
	self.actions = np.vstack((self.actions,a))
	#self.trainModel(self.states, self.actions)

    def saveModel(self):
        path = 'matrices/oldData/'
        currT = strftime("%Y-%m-%d %H:%M:%S", gmtime())

        inptFileOut = os.path.join(path, 'inputs' + currT + '.npy')
        stateFileOut = os.path.join(path, 'states' + currT + '.npy')

        np.save(stateFileOut, self.states)
	np.save(inptFileOut, self.actions)
        np.save(self.itrFile, self.itr)
