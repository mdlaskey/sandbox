from framework import *
import test_one_armed_robot
from Classes.robotParts import *
from Classes.oneArmedRobot import OneArmedRobot
from learner import *
import math
import numpy as np
import os, sys
import time

inst = test_one_armed_robot.GraspingWorld
trial = inst(False, None, False, False, None, None)