import cv2
import numpy as np
import const

# returns distance between loc1 and loc2
# locs - two-element tuples of integers x,y
def dist(loc1, loc2):
    return ((loc1[0] - loc2[0])**2 + (loc1[1] - loc2[1])**2)**(1.0/2.0)

# highlights a certain location with kernel
# used for debugging
# map - image np.array
# val - 3 element tuple value to replace at loc kernel
# loc - 2 element tuple of integers x,y
def highlight(map, val, loc):
    x, y = loc
    for t in range(-1, 2):
        for s in range(-1, 2):
            map[y + t, x + s] = val
    return map

# replace a point with val at loc
# map - image np.array
# val - 3 element tuple value to replace with
# loc - 2 element x,y
def point(map, val, loc):
    x, y = loc
    map[y, x] = val 
    return map

def findMaxRed(map):
    x,y = 0, 0
    for i in range(np.shape(map)[0]):
        for j in range(np.shape(map)[1]):
            b,g,r = [int(s) for s in map[j,i]]
            maxB, maxG, maxR = [int(t) for t in map[y,x]]
            if (r - g) > (maxR - maxG) and (r - b) > (maxR - maxB):
                x, y = i, j
    return map[j,i], (x, y)

#cv2.nameWindow('preview')
vc = cv2.VideoCapture(0)

if vc.isOpened():
    rval, frame = vc.read()
else:
    rval = False

rval, frame = vc.read()

# crop frame

frame = frame[0+const.OFFSET_Y:const.HEIGHT+const.OFFSET_Y, 0+const.OFFSET_X:const.WIDTH+const.OFFSET_X]    
blue, green, red = cv2.split(frame)

# identify the marker in the middle of the circle (may not be perfect)
#minRedVal, maxRedVal, minRedLoc, maxRedLoc = cv2.minMaxLoc(red) # the issue with this is it will pick up white and any color with high red value
maxRedVal, maxRedLoc = findMaxRed(frame)

#identify the darkest point in the image (which should be on the circle)
minGreenVal, maxGreenVal, minGreenLoc, maxGreenLoc = cv2.minMaxLoc(green)

# highlight those colors to show in image
res = cv2.cvtColor(green,cv2.COLOR_GRAY2RGB)
res = highlight(res, (0, 255, 0), minGreenLoc)
res = highlight(res, (0, 0, 255), maxRedLoc)

# get the distince between the extremae
d = dist(minGreenLoc, maxRedLoc)

f = open('hyperparams.txt', 'w+')
f.write(str(maxRedLoc[0]) + "," + str(maxRedLoc[1]) + "\n" + str(d))

cv2.imshow('frame', frame)
cv2.imshow('preview', res)

while 1:
    key = cv2.waitKey(20)
    if key == 27:
        break

vc.release()
cv2.destroyAllWindows()


