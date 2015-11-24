import cv2
import numpy as np
from utilities import const, recorder


class BinaryCamera(Object):
    
    tolerance = 2000                        # tolerance from distance around ring's border
    lower_green = [30,10,0]                 # table's lowest shade of green
    upper_green = [70,140,180]              # highest green
    lower_black = (0,0,0)                   # ring's lowest black
    upper_black = (130,130,130)             # highest black
    intermediate_green = [120, 180, 120]    # intermediate green value (between range)


    def __init__(self, meta='meta.txt', record=False):
        """
        meta -  path to file with calibration data (x,y\ndistance), e.g.
                212,207
                200.85
        """
        self.vc = cv2.videoCapture(0)
        self.recorder = None

        if vc.isOpened():
            rval, frame = vc.read()
        else:
            rval = False
        
        f = open(meta, 'r')
        self.maxRedLoc = [ int(x) for x in f.readline().split(',') ]
        self.d = int(float(f.readline()))
        self.d_squared = self.d * self.d
        f.close()
        
        if record:
            self.recorder = recorder.generate((const.HEIGHT, const.WIDTH))
    
    
    def read_frame(self, show=False):
        rval, frame = self.vc.read()
        frame = frame[0+const.OFFSET_Y:const.HEIGHT+const.OFFSET_Y, 0+const.OFFSET_X:const.WIDTH+const.OFFSET_X]

        # identify black ring on table
        for i in range( int(self.maxRedLoc[1] - 2*d), int(self.maxRedLoc[1] + 2*d), 2 ):
            for j in range( int(self.maxRedLoc[0] - 2*d), int(self.maxRedLoc[0] + 2*d), 2 ):
                if abs(BinaryCamera.dist_squared((j, i), maxRedLoc) - d_squared) < BinaryCamera.tolerance:
                    # map dark ring to a shade of green
                    BinaryCamera.mapRange(frame, (j, i))                                        
        
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_binary = cv2.inRange(frame_hsv, lower_green, upper_green)
        frame_binary = 255 - frame_binary
        frame_binary = cv2.medianBlur(frame_binary,7)

        if show:
            cv2.imshow("binary", frame_binary)
        if self.recorder:
            rec.write(cv2.cvtColor(mask_b, cv2.COLOR_GRAY2BGR))
            
        return frame_binary

    def destroy(self):
        cv2.destroyAllWindows()


    def dist(loc1, loc2):
        d_squared = dist_squared(loc1, loc2)
        return d_squared ** (.5)

    # attempting to reduce as much computation as possible
    def dist_squared(loc1, loc2):
        diffx = loc1[0] - loc2[0]
        diffy = loc1[1] - loc2[1]
        return diffx * diffx + diffy * diffy

    def mapRange(frame, loc):
        """
        Given a point, map pixel and surrounding pixels 
        to intermediate green if it is considered dark or black
        """
        x, y = loc
        blue, green, red = map[y,x]
        blueLow, greenLow, redLow = BinaryCamera.lower_black
        blueUp, greenUp, redUp = BinaryCamera.upper_black
        if greenLow < green \
            and greenUp > green \
            and blueLow < blue \
            and blueUp > blue \
            and redLow < red \
            and redUp > red:

            map[y, x] =  BinaryCamera.intermediate_green
            map[y+1, x] = BinaryCamera.intermediate_green
            map[y, x+1] = BinaryCamera.intermediate_green
            map[y+1, x+1] = BinaryCamera.intermediate_green

