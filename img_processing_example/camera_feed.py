import cv2
import numpy as np
import IPython

cv2.namedWindow("preview")
vc = cv2.VideoCapture(0)

if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False

OFFSET_X = 140
OFFSET_Y = 425
#fgbg = cv2.BackgroundSubtractorMOG2()
imgs = [] 

imgs = np.zeros([500,500,100])
I = 100

#Record a 100 frames to find threshold 
while 1:
	frame = frame[0+OFFSET_X:500+OFFSET_X, 0+OFFSET_Y:500+OFFSET_Y]
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	lower_green = np.array([10,10,0])
	upper_green = np.array([70,255,225])
	mask = cv2.inRange(hsv, lower_green, upper_green)
	mask = 255-mask
	mask_b = cv2.medianBlur(mask,7)
	cv2.imshow("preview", frame)
	cv2.imshow("bit",mask)
	cv2.imshow("bit_m",mask_b)
	rval, frame = vc.read()
	key = cv2.waitKey(20)
	if key == 27: # exit on ESC
	    break

print imgs

# #Take min and max over time 
# img_max = img(500,)
# for i in range[I]:






#IPython.embed()






cv2.destroyWindow("preview")
