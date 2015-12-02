import cv2
import numpy as np
from utilities import const

def dist(loc1, loc2):
    """
    Return euclidean distance between loc1 and loc2
    locs - two-element tuples of integers x, y
    """
    return ((loc1[0] - loc2[0])**2 + (loc1[1] - loc2[1])**2)**(1.0/2.0)

def highlight(map, val, loc):
    """
    highlights a certain location with kernel
    used for debugging
    map - image np.array
    val - 3 element tuple value to replace at loc kernel
    loc - 2 element tuple of integers x,y
    """
    x, y = loc
    for t in range(-1, 2):
        for s in range(-1, 2):
            map[y + t, x + s] = val
    return map

def point(map, val, loc):
    """
    replace a point with val at loc
    map - image np.array
    val - 3 element tuple value to replace with
    loc - 2 element x,y
    """
    x, y = loc
    map[y, x] = val 
    return map

def findMaxRed(map):
    """
    return the tuple RGB and tuple location of the most red
    point in the given np.array frame
    """
    x,y = 0, 0
    for i in range(np.shape(map)[0]):
        for j in range(np.shape(map)[1]):
            b,g,r = [int(s) for s in map[j,i]]
            maxB, maxG, maxR = [int(t) for t in map[y,x]]
            if (r - g) > (maxR - maxG) and (r - b) > (maxR - maxB):
                x, y = i, j
    return map[j,i], (x, y)


def eval(frame):
    """
    Find and evaluate the mins and maxes and distances
    Show filtered frames that contain highlighted points
    Return a tuple which contians a tuple location and float distance.
    """
    frame = frame[0+const.OFFSET_Y:const.HEIGHT+const.OFFSET_Y, 0+const.OFFSET_X:const.WIDTH+const.OFFSET_X]    
    blue, green, red = cv2.split(frame)
    
    # determine max reds and min greens
    maxRedVal, maxRedLoc = findMaxRed(frame)
    minGreenVal, maxGreenVal, minGreenLoc, maxGreenLoc = cv2.minMaxLoc(green)
    
    # highlight locations before displaying to screen
    res = cv2.cvtColor(green,cv2.COLOR_GRAY2RGB)
    res = highlight(res, (0, 255, 0), minGreenLoc)
    res = highlight(res, (0, 0, 255), maxRedLoc)
    
    d = dist(minGreenLoc, maxRedLoc)
    
    cv2.imshow('preview', res)

    return maxRedLoc, d

vc = cv2.VideoCapture(0)
if vc.isOpened():
    rval, frame = vc.read()
else:
    rval = False


while 1:
    rval, frame = vc.read()
    
    maxRedLoc, d = eval(frame)
    key = cv2.waitKey(20)
    if key == 27:
        break

f = open('meta.txt', 'w')
f.write(str(maxRedLoc[0]) + "," + str(maxRedLoc[1]) + "\n" + str(d))

cv2.release()
cv2.destroyAllWindows
