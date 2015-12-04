import cv2
import numpy as np
import constants


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
    blue, green, red = frame[y,x]
    blueLow, greenLow, redLow = BinaryCamera.lower_black
    blueUp, greenUp, redUp = BinaryCamera.upper_black
    if greenLow < green \
        and greenUp > green \
        and blueLow < blue \
        and blueUp > blue \
        and redLow < red \
        and redUp > red:

        frame[y, x] =  BinaryCamera.intermediate_green
        frame[y+1, x] = BinaryCamera.intermediate_green
        frame[y, x+1] = BinaryCamera.intermediate_green
        frame[y+1, x+1] = BinaryCamera.intermediate_green


class BinaryCamera():
    
    tolerance = 2000                        # tolerance from distance around ring's border
    lower_green = np.array([30,10,0])       # table's lowest shade of green
    upper_green = np.array([70,140,180])    # highest green
    lower_black = (0,0,0)                   # ring's lowest black
    upper_black = (130,130,130)             # highest black
    intermediate_green = [120, 180, 120]    # intermediate green value (between range)


    def __init__(self, meta=''):
        """
        meta -  path to file with calibration data (x,y\ndistance), e.g.
                212,207
                200.85
        """
        self.vc = None
        self.recorder = None

        #if self.vc.isOpened():
        #    rval, frame = self.vc.read()
        #else:
        #    rval = False
        if not meta == '':
            f = open(meta, 'r')
            self.maxRedLoc = [ int(x) for x in f.readline().split(',') ]
            self.d = int(float(f.readline()))
            self.d_squared = self.d * self.d
            f.close()
        
    def open(self):
        self.vc = cv2.VideoCapture(0)
        
    def close(self):
        if self.is_open():
            self.vc.release()
    
    def is_open(self):
        return self.vc is not None and self.vc.isOpened()

    def read_frame(self, show=False):
        """
        Returns cropped frame of raw video
        """
        rval, frame = self.vc.read()
        frame = frame[0+constants.OFFSET_Y:constants.HEIGHT+constants.OFFSET_Y, 0+constants.OFFSET_X:constants.WIDTH+constants.OFFSET_X]
        if show:
            cv2.imshow("preview", frame)
        return frame
        
    def read_binary_frame(self, show=False):
        """
        Returns a cropped binary frame of the video
        Significantly slower than read_frame due to the pipeline.
        """
        rval, frame = self.vc.read()
        frame = frame[0+constants.OFFSET_Y:constants.HEIGHT+constants.OFFSET_Y, 0+constants.OFFSET_X:constants.WIDTH+constants.OFFSET_X]        
        frame_binary = self.pipe(frame)
        
        if show:
            cv2.imshow("binary", frame_binary)
            
        return frame_binary
        
    def pipe(self, frame):
        """
        sends frame through cv2 pipeline to render
        binary image of original frame. Assumes calibration
        """
        # identify black ring on table        
        for i in range( int(self.maxRedLoc[1] - 2*self.d), int(self.maxRedLoc[1] + 2*self.d), 3 ):
            for j in range( int(self.maxRedLoc[0] - 2*self.d), int(self.maxRedLoc[0] + 2*self.d), 3 ):
                if abs(dist_squared((j, i), self.maxRedLoc) - self.d_squared) < BinaryCamera.tolerance:
                    # map dark ring to a shade of green
                    mapRange(frame, (j, i))   

        frame = cv2.resize(frame, (250, 250))                 
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_binary = cv2.inRange(frame_hsv, BinaryCamera.lower_green, BinaryCamera.upper_green)
        frame_binary = 255 - frame_binary
        frame_binary = cv2.medianBlur(frame_binary,7)
    
        return frame_binary

    def destroy(self):
        cv2.destroyAllWindows()



