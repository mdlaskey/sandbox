import datetime
import os
import cv2

class Dataset():

    def __init__(self, name, path, options):
        self.name = name
        self.options = options
        self.path = options.datasets_dir + self.name + "/"
        self.writer = open(self.path + "controls.txt", 'a')
        self.i = Dataset.next_datum_index(self.path)


    def put(self, frame, controls):
        filename = "img_" + str(self.i) + ".jpg"
        filepath = self.path + filename
        if os.path.isfile(filepath):
            self.i += 1
            self.put(frame, controls)
        else:
            controls_str = Dataset.controls2str(controls)
            self.writer.write(filename + controls_str + '\n')
            cv2.imwrite(self.path + filename, frame)
            self.i += 1

    def get(self, filename):
        """ return cv2 image with filename and list of controls """
        filepath = self.path + filename
        if not os.path.isfile(filepath):
            raise Exception("File not found at " + filepath)
        return cv2.imread(filepath)
        


    @staticmethod
    def next_datum_index(path, i=0):
        filepath = path + "img_" + str(i) + ".jpg"
        while os.path.isfile(filepath):
            i += 1
            filepath = path + "img_" + str(i) + ".jpg"
        return i

    @staticmethod
    def create_ds(options, prefix=""):
        name = prefix + "_" + datetime.datetime.now().strftime("%m-%d-%Y_%Hh%Mm%Ss") + "/"
        path = options.datasets_dir + name
        os.makedirs(path)
        ds = Dataset(name, path, options)
    
        about = open(ds.path + "about.txt", 'a+')
        about.write(datetime.datetime.now().strftime('%m-%d-%Y %H:%M:%S'))
        about.write("\n\nnet: " + options.model_path + "\nweights: " + options.weights_path)

        return ds

    @staticmethod
    def get_ds(options, name):
        path = options.datasets_dir + name
        ds = Dataset(name, path, options)
        if not os.path.exists(ds.path):
            raise Exception("No dataset found at " + path)
        return ds

    @staticmethod
    def controls2str(controls):
        """ Returns space separated controls with space preceding all controls """
        controls_str = ""
        for c in controls:
            controls_str += " " + str(c)
        return controls_str
