import numpy as np
import cv2

OFFSET_Y = 60
OFFSET_X = 160


img = cv2.imread('in/topview1-r.jpg')
img = img[0+OFFSET_Y:250+OFFSET_Y, 0+OFFSET_X:270+OFFSET_X]

gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
lower_green = np.array([30, 50, 30])
upper_green = np.array([175,  192,  183])
thresh = cv2.inRange(img, lower_green, upper_green)
thresh = 255-thresh


#noise removal
kernel = np.ones((3,3),np.uint8)
opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)

# sure background area
sure_bg = cv2.dilate(opening,kernel,iterations=30)
 
# Finding sure foreground area
dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,3)
ret, sure_fg = cv2.threshold(dist_transform,0.3*dist_transform.max(),255,0)

# Finding unknown region
sure_fg = np.uint8(sure_fg)
unknown = cv2.subtract(sure_bg,sure_fg)

ret, markers = cv2.connectedComponents(sure_fg)
 
# Add one to all labels so that sure background is not 0, but 1
markers = markers+1
 
# Now, mark the region of unknown with zero
markers[unknown==255] = 0

cv2.imshow("markers", markers)

markers = cv2.watershed(img,markers)
img[markers == -1] = [255,0,0]

cv2.imshow("thresh", thresh)
cv2.imshow("unknown", unknown)
cv2.imshow("sure_bg", sure_bg)
cv2.imshow("result", img)

while 1:
    key = cv2.waitKey(20)
    if key == 27: # exit on ESC
	    break

cv2.destroyWindow("preview")

