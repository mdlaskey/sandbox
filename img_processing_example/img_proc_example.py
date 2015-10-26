import math
import numpy as np
import cv2
import IPython

im_gray= cv2.imread('box2d_example.png', cv2.CV_LOAD_IMAGE_GRAYSCALE)

(thresh, im_bw) = cv2.threshold(im_gray, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
im_bw = cv2.pyrDown(im_bw)
im_bw = cv2.pyrDown(im_bw)

cv2.imwrite('bw_image.png', im_bw)
