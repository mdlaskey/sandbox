from pipeline.bincam import BinaryCamera
import numpy as np
import caffe
import constants
import time
import cv2

def test(options, c, izzy, t):
    """
    params - options, controller, izzy, turntable
    """
    bincam = BinaryCamera()
    bincam.open()

    while True:
        frame = bincam.read_binary_frame(show=options.show)
        controls = c.getUpdates()
        print "test: " + str(controls)
        if controls is None:
            print "Done"
            izzy.stop()
            break
        controls[1] = 0
        controls[3] = 0
        izzy.control(controls)
        t.control([controls[5]])
        time.sleep(.05)



def deploy(options, c, izzy, t):
    """
    params - options, controller, izzy, turntable    
    """
    bincam = bincam.BinaryCamera()
    bincam.open()
    frame = bincam.read_binary_frame()
    net = caffe.Net(options.model, options.weights, caffe.TEST)
    
    dataset_path = create_new_dataset()
    writer = open(dataset_path + "controls.txt", 'w+')
    i = 0

    while True:
        if c.shouldOverride():
            controls = c.getUpdates()
            controls[1] = 0
            controls[3] = 0
            print controls
            izzy.control(controls)
            t.control([controls[5]])
            
            simpleControls = [controls[0], controls[2], controls[4], controls[5]]
            if not all(int(sc)==0 for sc in simpleControls):
                frame = bincam.read_frame(show=options.show)                
                filename = "img_" + str(i) + ".jpg"
                save_example(writer, dataset_path, filename, frame, simpleControls)
                i+=1
            
        else:
            # bincam frames go from 0 to 255 in 1 dim
            # assuming 125x125 images
            data4D = np.zeros([1, 3, 125, 125])
            frame = frame / 255.0
            data4D[0,0,:,:] = frame
	    data4D[0,1,:,:] = frame
	    data4D[0,2,:,:] = frame
          
            net.forward_all(data=data4D)
            controls = net.blobs['out'].data.copy()[0]
            
            # scale controls and squash small controls
            controls = revert_controls(controls)
            print controls
            izzy.control(controls)
            t.control([controls[5]])
            frame = bincam.read_binary_frame()
            
        time.sleep(.05)
    


def learn(options, c, izzy, t):
    """
    params - options, controller, izzy, turntable    
    """
    bincam = bincam.BinaryCamera()
    bincam.open()

    dataset_path = create_new_dataset()
    writer = open(dataset_path + "controls.txt", 'w+')

    i = 0
    while True:
        controls = c.getUpdates()     
        print controls
        
        controls[1] = 0
        controls[3] = 0
        izzy.control(controls)
        t.control([controls[5]])
        
        # store this however you please. (concatenate into array?)
        simpleControls = [controls[0], controls[2], controls[4], controls[5]]
        if not all(int(sc)==0 for sc in simpleControls):
            frame = bincam.read_frame(show=options.show)            
            filename = "img_" + str(i) + ".jpg"
            save_example(writer, dataset_path, filename, frame, simpleControls)
            i+=1
        time.sleep(.05)


def create_new_dataset():
    i = 0
    datasets = constants.ROOT + "datasets/"
    while os.path.exists(datasets + "dataset" + str(i) + "/"):
        i += 1
    dataset_path = datasets + "dataset" + str(i) + "/"
    os.makedirs(dataset_path)
    return dataset_path

def save_example(writer, dataset_path, filename, frame, controls):
    cv2.imwrite(dataset_path + filename, frame)
    controls_string = ""
    for c in controls:
        controls_string += " " + str(c)
    writer.write(filename + controls_string + '\n')



def revert_controls(controls):
    for i in range(len(controls)):
        controls[i] = (controls[i] - translations[i]) * scales[i] * 1.2
        if abs(controls[i]) < drift:
            controls[i] = 0.0
    return [controls[0], 0.0, controls[1], 0.0, controls[2], controls[3]]

def transform_controls(controls):
    controls = [controls[0], controls[2], controls[4], controls[5]]
    for i in range(len(controls)):
        controls[i] = controls[i] / scales[i] + translations[i]
    return controls
 

