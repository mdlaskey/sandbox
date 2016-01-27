import sys
import tty, termios
from options import Options
from gripper.TurnTableControl import *
from gripper.PyControl import *
from gripper.xboxController import *
from pipeline.bincam import BinaryCamera
from Net.tensor import net2, inputdata
import datetime
import time

def getch():
    """
        getch will cause python program to wait until
        command is given rather than constantly checking
        from updates while looping.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class AMT():

    SAVE_PATH = Options.

    def __init__(self, bincam, izzy, turntable, controller, options=Options()):
        self.bc = bincam
        self.izzy = izzy
        self.turntable = turntable
        self.c = controller
        self.options = options

        self.recording = []
        self.states = []

    def rollout_tf(self, iterations=50):
        net = self.options.tf_net
        path = self.options.tf_path
        sess = net.load(var_path=path)
        try:
            for i in range(iterations):
                state = self.izzy.getState()
                frame = self.bc.read_binary_frame()
                self.recording.append(frame, state)
                frame = np.reshape(frame, (125, 125, 1))
                net_controls = net.output(sess, frame)
                controls = self.net2controls(net_controls)
                self.update_gripper(controls)
                time.sleep(.03)
        except KeyboardInterrupt:
            pass
        sess.close()        
        self.save_recordings()

    # TODO: update inputdata to not load all images into memory, just one batch at a time
    def update_weights(self, iterations=10):
        net = self.options.tf_net
        path = self.options.tf_path
        data = inputdata.InputData(self.options.train_path, self.options.test_path)
        net.optimize(iterations, batch_size=300, path=path,  data=data)

    # TODO: set options paths and transfer dataset aggregation code here (scripts/dagger.py.
    def segment(self):
        raise NotImplementedError

    def save_recordings(self):
        path = self.options.amt_dir + datetime.datetime.now().strftime("%m-%d-%Y_%Hh%Mm%Ss") + '/'
        print "Saving to " + path + "..."
        os.makedirs(path)
        i = 0
        f = open(path + 'states.txt', 'w+')
        for frame, state in zip(self.recording, self.states):
            name = "frame_" + str(i) + ".jpg"
            cv2.imwrite(path + name, frame)
            f.write(name + self.lst2str(state) + "\n") 
            i+=1
        f.close()
        self.recording = []
        self.states = []


    def lst2str(self, lst):
        s = ""
        for el in lst:
            lst.append(" " + str(el))
        return s


bincam = BinaryCamera('./meta.txt')
bincam.open()

options = Options()

t = TurnTableControl() # the com number may need to be changed. Default of com7 is used
izzy = PyControl(115200, .04, [0,0,0,0,0],[0,0,0]); # same with this
c = XboxController([options.scales[0],155,options.scales[1],155,options.scales[2],options.scales[3]])

options.tf_net = net2.NetTwo()
options.tf_path = options.tf_dir + 'net2/net2_01-21-2016_02h14m08s.ckpt'
options.tf_net = net3.NetThree()
options.tf_path = options.tf_dir + 'net3/net3_01-19-2016_00h47m49s.ckpt'

# TODO: set actual train/test paths
options.train_path = 'path/to/train.txt'
options.test_path = 'path/to/test.txt'
# TODO: set path to raw image directory and segmented image directory
options.raw_path = 'path/to/raw_images/'
options.seg_path = 'path/to/seg_images/'



amt = AMT(bincam, izzy, t, c, options=options)

while True:
    print "Waiting for keypress ('q' -> quit, 'r' -> rollout, 'u' -> update weights): "
    char = getch()
    if char == 'q':
        print "Quitting..."
        break
    
    elif char == 'r':
        print "Rolling out..."
        amt.rollout_tf()
        print "Done rolling out."

    elif char == 'u':
        print "Updating weights..."
        amt.update_weights()
        print "Done updating."

    elif char == 'b':
        print "Binary-segmenting..."
        
        print "Done segmenting."

options.tf_sess.close()
print "Done."
