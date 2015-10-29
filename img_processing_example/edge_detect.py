import math
import numpy as np
import cv2
import IPython
import segment

import sys

full_name = sys.argv[1]

filename, ext = full_name.split('.')
ext = '.' + ext

#cv2.IMREAD_GRAYSCALE
img = cv2.imread('in/' + filename + ext)
img = segment.hist_equalize(img)
img = segment.blur_color(img)
img = segment.gray(img)
#img = thresh(img)
img = segment.edges(img)
img = segment.dilate(img)
cv2.imwrite('out/' + filename + '_edges' + ext, img)


