"""parse files"""

import os
import re
import numpy as np
import pickle
from os import path

class Parser():
    def __init__(self):
        path = os.getcwd()
        self.files = os.listdir(path)
        self.files.sort()
        
        self.regItr = 'itr\d.npy'
        self.regResults = 'results\d.npy'

    def parse(self, sub):
        iFiles = []
        rFiles = []
        for f in self.files:
            matchI = re.match(self.regItr, f)
            matchR = re.match(self.regResults, f)
            if matchI:
                iFiles.append(f)
            if matchR:
                rFiles.append(f)

        i = np.array([np.load(itr) for itr in iFiles])
        r = np.array([np.load(results) for results in rFiles])
        iName = 'itrTotal' + sub + '.npy'
        rName = 'resultsTotal' + sub + '.npy'
        pickle.dump(i, open(iName, 'wb'))
        pickle.dump(r, open(rName, 'wb'))
