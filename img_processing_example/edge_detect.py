import math
import numpy as np
import cv2
import IPython
import segment


#cv2.IMREAD_GRAYSCALE
filename = 'frame0003'
ext = '.jpg'
img = cv2.imread(filename + ext)
img = segment.hist_equalize(img)
img = segment.blur_color(img)
img = segment.gray(img)
#img = thresh(img)
img = segment.edges(img)
img = segment.dilate(img)
cv2.imwrite(filename + '_edges' + ext, img)


