import cv2
import numpy as np


def dist(loc1, loc2):
    return ((loc1[0] - loc2[0])**2 + (loc1[1] - loc2[1])**2)**(1.0/2.0)

def highlight(map, val, loc):
    x, y = loc
    for t in range(-1, 2):
        for s in range(-1, 2):
            map[y + t, x + s] = val

    return map

def point(map, val, loc):
    x, y = loc
    map[y, x] = val 
    return map

def mapRange(map, lower, upper, loc):
    x, y = loc
    blue, green, red = map[y, x]
    blueLow, greenLow, redLow = lower
    blueUp, greenUp, redUp = upper
    
    if blueLow < blue \
            and blueUp > blue \
            and redLow < red \
            and redUp > red \
            and greenLow < green \
            and greenUp > green:
        map[y, x] = 50, 180, 50
    return map

"""
cv2.nameWindow('preview')
vc = cv2.VideoCapture(0)

if vc.isOpened():
    rval, frame = vc.read()
else:
    rval = False
"""
frame = cv2.imread('base.jpg')

OFFSET_Y = 54
OFFSET_X = 160

#crop the frame
frame = frame[0+OFFSET_Y:250+OFFSET_Y, 0+OFFSET_X:250+OFFSET_X]
blue, green, red = cv2.split(frame)

# identify the marker in the middle of the circle (may not be perfect)
minRedVal, maxRedVal, minRedLoc, maxRedLoc = cv2.minMaxLoc(red)

#identify the darkest point in the image (which should be on the circle)
minGreenVal, maxGreenVal, minGreenLoc, maxGreenLoc = cv2.minMaxLoc(green)

# highlight those colors to show in image
res = cv2.cvtColor(green,cv2.COLOR_GRAY2RGB)
res = highlight(res, (0, 255, 0), minGreenLoc)
res = highlight(res, (0, 0, 255), maxRedLoc)

# get the distince between the extremae
d = dist(minGreenLoc, maxRedLoc)

# test image
im = cv2.imread('topview_black1.jpg')
im = im[0+OFFSET_Y:250+OFFSET_Y, 0+OFFSET_X:250+OFFSET_X]


# process those pixels within the circle
# and map when within a specific range (dark colors)
for i in range(np.shape(res)[0]):
    for j in range(np.shape(res)[1]):
        if abs(dist((j, i), maxRedLoc) - d) < 10:
            im = mapRange(im, (0,0,0), (100, 100, 100), (j, i))


hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
lower_green = np.array([30,10,0])
upper_green = np.array([90,255,225])
mask = cv2.inRange(hsv, lower_green, upper_green)
mask = 255 - mask
mask_b = cv2.medianBlur(mask,7)

cv2.imshow('original', im)
cv2.imshow('hsv', hsv)
cv2.imshow("preview", mask_b)

while 1:
    key = cv2.waitKey(20)
    if key == 27:
        break

cv2.destroyWindow('preview')


