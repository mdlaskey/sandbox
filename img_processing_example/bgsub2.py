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

proc = np.zeros(base.shape)


for y in range(0, base.shape[0]):
    for x in range(0, base.shape[1]):
        proc[y][x] = base[y][x] - img[y][x]
        diff = base[y][x] - img[y][x]
