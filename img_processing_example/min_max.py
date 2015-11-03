import cv2
import numpy as np
import IPython
import segment

OFFSET_Y = 180
OFFSET_X = 480

def replace_lower(curr, new):
    if curr[0] > new[0] and curr[1] > new[1] and curr[2] > new[2]:
        return np.array(new)
    return np.array(curr)


def replace_upper(curr, new):
    if curr[0] < new[0] and curr[1] < new[1] and curr[2] < new[2]:
        return np.array(new)
    return np.array(curr)

upper = np.array([0, 0, 0])
lower = np.array([255, 255, 255])


"""
cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

I = 100


test = frame[0+OFFSET_Y:440+OFFSET_Y, 0+OFFSET_X:440+OFFSET_X]

cv2.imshow('preview', test)    # just showing one image as test
while 1:
    key = cv2.waitKey(20)
    if key == 27:
        break
cv2.destroyWindow("preview")



for i in range(I):
    frame = frame[0+OFFSET_Y:440+OFFSET_Y, 0+OFFSET_X:440+OFFSET_X]
    cv2.imshow("preview", frame)

    blue, green, red = cv2.split(frame) # blue green red for cv2 not rgb    

    blueMinVal, blueMaxVal, _, _ = cv2.minMaxLoc(blue)
    greenMinVal, greenMaxVal, _, _ = cv2.minMaxLoc(green)
    redMinVal, redMaxVal, _, _ = cv2.minMaxLoc(red)

    lower = replace_lower(lower, [blueMinVal, greenMinVal, redMinVal])
    upper = replace_upper(upper, [blueMaxVal, greenMaxVal, redMaxVal])

    rval, frame = vc.read()


print lower
print upper



"""
OFFSET_Y = 60
OFFSET_X = 160

frame = cv2.imread('in/base.jpg')
# in practice, adjust this a little to get enitre rim
frame = frame[0+OFFSET_Y:250+OFFSET_Y, 0+OFFSET_X:270+OFFSET_X]

blue, green, red = cv2.split(frame) #cv2 has bgr channels not rgb

blueMinVal, blueMaxVal, _, _ = cv2.minMaxLoc(blue)
greenMinVal, greenMaxVal, _, _ = cv2.minMaxLoc(green)
redMinVal, redMaxVal, _, _ = cv2.minMaxLoc(red)

cv2.imshow('preview', frame)

lower = replace_lower(lower, [blueMinVal, greenMinVal, redMinVal])
upper = replace_upper(upper, [blueMaxVal, greenMaxVal, redMaxVal])

print upper
print lower


frame2 = cv2.imread('in/topview1-r.jpg')
frame2 = frame2[0+OFFSET_Y:250+OFFSET_Y, 0+OFFSET_X:270+OFFSET_X]
mask = cv2.inRange(frame2, lower, upper)
mask = 255-mask
mask_b = cv2.medianBlur(mask,3)
cv2.imshow('filtered', mask)
cv2.imshow('median', mask_b)


while 1:
    key = cv2.waitKey(20)
    if key == 27:
        break
cv2.destroyWindow("preview")

