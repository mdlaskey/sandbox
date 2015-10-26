import math
import numpy as np
import cv2
import IPython


def hist_equalize(img):
    hist,bins = np.histogram(img.flatten(),256,[0,256])

    cdf = hist.cumsum()
    cdf_normalized = cdf * hist.max() / cdf.max()

    cdf_m = np.ma.masked_equal(cdf,0)
    cdf_m = (cdf_m - cdf_m.min())*255/(cdf_m.max()-cdf_m.min())
    cdf = np.ma.filled(cdf_m,0).astype('uint8')
    img = cdf[img]
    img = cv2.fastNlMeansDenoisingColored(img,None,20,10,7,21)
    
    return img

def gray(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def distance_transform(img):
    return cv2.distanceTransform(img,cv2.DIST_L2,5)


def sure_fg(img):
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_OPEN,kernel, iterations = 2)
    surebg = cv2.dilate(opening,kernel,iterations=3)
    dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
    ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
    sure_fg = np.uint8(sure_fg)
    
    return sure_fg
def blur(img):
    return cv2.fastNlMeansDenoising(img,None,30, 7, 21)
def blur_color(img):
    return cv2.fastNlMeansDenoisingColored(img,None,5,5,7,21)
   
def edges(img):
    return cv2.Canny(img, 5, 100)
    
def thresh(img):
    img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 1)
    return img

def dilate(img):
    kernel = np.ones((3, 3), np.uint8)
    return cv2.dilate(img,kernel,iterations = 1)

#cv2.IMREAD_GRAYSCALE
#im_gray= cv2.imread('frame0000.jpg', cv2.IMREAD_COLOR)
"""img = cv2.imread('frame0003.jpg')
img = hist_equalize(img)
img = blur_color(img)
img = gray(img)
#img = thresh(img)
img = edges(img)
img = dilate(img)
cv2.imwrite('frame0003_out.jpg', img)
"""

img = cv2.imread('frame0003.jpg')
img = hist_equalize(img)
img = blur_color(img)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# setting bounds for green color
lower_green = np.array([36,100,50])
upper_green = np.array([81,255,255])

mask = cv2.inRange(hsv, lower_green, upper_green)
cv2.imwrite('frame0003_out.jpg', mask)
