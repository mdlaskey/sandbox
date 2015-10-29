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

# defined boundry size for camera mounted in Soda
img = img[50:300, 120:450]
img = segment.hist_equalize(img)
img = segment.blur_color(img)

#write out blurred version for debugging
cv2.imwrite('out/' + filename + '_blur' + ext, img)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# setting bounds for green color
lower_green = np.array([30,20,10])
upper_green = np.array([70,255,225])
mask = cv2.inRange(hsv, lower_green, upper_green)
mask = 255 - mask
cv2.imwrite('out/' + filename + '_thresh' + ext, mask)
