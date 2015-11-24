import cv2
import numpy as np
from utilities import const, recorder


class BinaryCamera(Object):
    
    tolerance = 2000                        # tolerance from distance around ring's border
    lower_green = np.array([30,10,0])       # table's lowest shade of green
    upper_green = np.array([70,140,180])    # highest shade

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
        for i in range( int(maxRedLoc[1] - 2*d), int(maxRedLoc[1] + 2*d), 2 ):
            for j in range( int(maxRedLoc[0] - 2*d), int(maxRedLoc[0] + 2*d), 2 ):
                if abs(dist_squared((j, i), maxRedLoc) - d_squared) < BinaryCamera.tolerance:
                    # map dark ring to a shade of green
                    mapRange(frame, (j, i))                                        
        
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_binary = cv2.inRange(frame_hsv, lower_green, upper_green)
        frame_binary = 255 - frame_binary
        frame_binary = cv2.medianBlur(frame_binary,7)

        if show:
            cv2.imshow("binary", frame_binary)
        if self.recorder:
            rec.write(cv2.cvtColor(mask_b, cv2.COLOR_GRAY2BGR))
            
        return frame_binary

    def destroy():
        cv2.destroyAllWindows()
