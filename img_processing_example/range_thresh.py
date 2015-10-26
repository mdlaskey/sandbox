import math
import numpy as np
import cv2
import IPython
import segment

filename = 'frame0003'
ext = '.jpg'

img = cv2.imread('in/' + filename + ext)
img = segment.hist_equalize(img)
img = segment.blur_color(img)

#write out blurred version for debugging
cv2.imwrite('out/' + filename + '_blur' + ext, img)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# setting bounds for green color
lower_green = np.array([36,100,50])
upper_green = np.array([81,255,255])

mask = cv2.inRange(hsv, lower_green, upper_green)
cv2.imwrite('out/' + filename + '_thresh' + ext, mask)

