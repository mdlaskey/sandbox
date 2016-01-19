"""
Script that test runs gripper, show camera view with "python scripts/test.py -s"
"""

from gripper.TurnTableControl import *
from gripper.PyControl import *
from gripper.xboxController import *

from pipeline.bincam import BinaryCamera
from options import Options
from lfd import LFD

import argparse
ap = argparse.ArgumentParser()
ap.add_argument('-s', '--show', required=False, action='store_true')
args = vars(ap.parse_args())


bincam = BinaryCamera('./meta.txt')
bincam.open()

options = Options()
options.show = args['show']

t = TurnTableControl() # the com number may need to be changed. Default of com7 is used
izzy = PyControl("/dev/cu.usbmodem14111",115200, .04, [0,0,0,0,0],[0,0,0]); # same with this
c = XboxController([options.scales[0],155,options.scales[1],155,options.scales[2],options.scales[3]])

lfd = LFD(bincam, izzy, t, c, options=options)
lfd.test()
