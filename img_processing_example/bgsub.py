import math
import numpy as np
import cv2
import IPython
import segment

import sys


full_name = sys.argv[1] 

filename, ext = full_name.split('.')
ext = '.' + ext

img = cv2.imread('in/' + filename + ext)
img = img[50:300, 120:450]

base = cv2.imread('in/base.jpg')
base = base[50:300, 120:450]

fgbg = cv2.createBackgroundSubtractorMOG2()

fgmask = fgbg.apply(base)
fgmask = fgbg.apply(img)

