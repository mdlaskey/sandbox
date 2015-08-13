""" main class for Robot Arm """

from framework import *
from test_polygon_sam_latest import *
import math
import numpy as np
import time


trials = 5
itr = 760

poly = PolygonDemo
main(poly)
poly.hey()

'''for i in range(1):
#for i in range(1,6):
    j = 0
    instance = PolygonDemo
    main(instance)
    trial = getInstance()
    trial.start(True)    

    
    while trial.getLengthStates() <= itr:
        if trial.getLengthStates() == itr-1:
            states = 
        
        pygame.event.post(pygame.QUIT)
            j+=1'''

