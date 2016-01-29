# TODO: rollout directories read and write
# TODO: caffe train and rollout

import sys
import tty, termios
from options import Options, AMTOptions
from gripper.TurnTableControl import *
from gripper.PyControl import *
from gripper.xboxController import *
from pipeline.bincam import BinaryCamera
from Net.tensor import net2, inputdata, net3
import datetime
import time
import os
import cv2

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

    def __init__(self, bincam, izzy, turntable, controller, options=AMTOptions()):
        self.bc = bincam
        self.izzy = izzy
        self.turntable = turntable
        self.c = controller
        self.options = options

        self.recording = []
        self.states = []

    def rollout_caffe(self, iterations=50):
        raise NotImplementedError

    # TODO: write method to handle converting between four dim state space and six dim state space
    def rollout_tf(self, iterations=50):
        net = self.options.tf_net
        path = self.options.tf_net_path
        sess = net.load(var_path=path)
        try:
            for i in range(iterations):
                curr_izzy_state = self.izzy.getState()
                curr_t_state = self.turntable.getState()
                frame = self.bc.read_binary_frame()
                self.recording.append((frame, state))
                frame = np.reshape(frame, (125, 125, 1))
                
                delta_state = net.output(sess, frame)
                new_izzy, new_t = self.delta2state(delta_state)
                
                print new_izzy, new_t
                # TODO: uncomment these to update izzy and t
                #self.izzy.sendStateRequest(new_izzy)
                #self.turntable.sendStateRequest(new_t)

                time.sleep(.03)
                
        except KeyboardInterrupt:
            pass
        sess.close()
        self.prompt_save()

    def prompt_save(self):
        num_rollouts = len(AMT.rollout_dirs())
        print "There are " + str(num_rollouts) + " rollouts. Save this one? (y/n): "
        char = getch()
        if char == 'y':
            return self.save_recording()
        elif char == 'n':
            return None
        self.prompt_save()

    def delta2state(self, delta_state):
        curr_state = self.izzy.getState()
        if not (curr_state[0] + delta_state[0] > self.options.ROTATE_UPPER_BOUND or curr_state[0] + delta_state[0] < self.options.ROTATE_LOWER_BOUND):
            curr_state[0] += delta_state[0]
        curr_state[2] += delta_state[1]
        curr_state[4] += delta_state[2]
        t_state = self.turntable.getState()
        t_state[0] += delta_state[3]
        return curr_state, t_state

    # TODO: update inputdata to not load all images into memory, just one batch at a time
    def update_weights(self, iterations=10):
        net = self.options.tf_net
        path = self.options.tf_net_path
        data = inputdata.AMTData(self.options.train_file, self.options.test_file)
        net.optimize(iterations, batch_size=300, path=path,  data=data)

    # TODO: set options paths and transfer dataset aggregation code here (scripts/dagger.py.
    def segment(self):
        raise NotImplementedError

    def save_recording(self):
        """
        Save instance recordings and states by writing filename and corresponding state
        to states files and writing images to master frames dir and appropriate rollout dir.
        Clear recordings and states from memory when done writing
        :return:
        """
        rollout_name = self.next_rollout()
        rollout_path = self.options.rollouts_dir + rollout_name + '/'

        print "Saving rollout to " + rollout_path + "..."
        print "Saving raw frames to " + self.options.originals_dir + "..."

        os.makedirs(rollout_path)
        raw_states_file = open(self.options.originals_dir + "states.txt", 'a+')
        rollout_states_file = open(rollout_path + "states.txt", 'a+')

        i = 0
        for frame, state in zip(self.recording, self.states):
            filename = rollout_name + "_frame_" + str(i) + ".jpg"
            raw_states_file.write(filename + self.lst2str(state))
            rollout_states_file.write(filename + self.lst2str(state))
            cv2.imwrite(self.options.originals_dir + filename, frame)
            cv2.imwrite(rollout_path + filename, frame)
            i += 1

        raw_states_file.close()
        rollout_states_file.close()
        self.recording = []
        self.states = []
        print "Done saving."

    @staticmethod
    def rollout_dirs():
        """
        :return: list of strings that are the names of rollout dirs
        """
        return list(os.walk(AMTOptions.rollouts_dir))[0][1]

    @staticmethod
    def next_rollout():
        """
        :return: the String name of the next new potential rollout
                (i.e. do not overwrite another rollout)
        """
        i = 0
        prefix = AMTOptions.rollouts_dir + 'rollout'
        path = prefix + str(i) + "/"
        while os.path.exists(path):
            i += 1
            path = prefix + str(i) + "/"
        return 'rollout' + str(i)

    @staticmethod
    def lst2str(lst):
        """
        returns a space separated string of all elements. A space
        also precedes the first element.
        :param lst:
        :return:
        """
        s = ""
        for el in lst:
            lst.append(" " + str(el))
        return s




if __name__ == "__main__":
    bincam = BinaryCamera('./meta.txt')
    bincam.open()

    options = AMTOptions()

    t = TurnTableControl() # the com number may need to be changed. Default of com7 is used
    izzy = PyControl(115200, .04, [0,0,0,0,0],[0,0,0]) # same with this
    #c = XboxController([options.scales[0],155,options.scales[1],155,options.scales[2],options.scales[3]])
    c = None
    #options.tf_net = net2.NetTwo()
    #options.tf_net_path = options.tf_dir + 'net2/net2_01-21-2016_02h14m08s.ckpt'
    options.tf_net = net3.NetThree()
    options.tf_net_path = options.tf_dir + 'net3/net3_01-19-2016_00h47m49s.ckpt'

    # TODO: set actual train/test paths
    options.train_path = 'path/to/train.txt'
    options.test_path = 'path/to/test.txt'
    # TODO: set path to raw image directory and segmented image directory
    options.raw_path = 'path/to/raw_images/'
    options.seg_path = 'path/to/seg_images/'

    amt = AMT(bincam, izzy, t, c, options=options)

    while True:
        print "Waiting for keypress ('q' -> quit, 'r' -> rollout, 'u' -> update weights, 'b' -> binary segment): "
        char = getch()
        if char == 'q':
            print "Quitting..."
            break
        
        elif char == 'r':
            print "Rolling out..."
            ro = amt.rollout_tf()
            print "Done rolling out."

        elif char == 'u':
            print "Updating weights..."
            amt.update_weights()
            print "Done updating."



    print "Done."
