"""
Class variables are typical constants (do not change externally)
Instance variables are options (ok to change case by case externally)

This is modeled after leveldb's options
"""

class Options():
    translations = [0.0, 0.0, 0.0, 0.0]
    scales = [40.0, 120.0, 90.0, 100.0]
    drift = 20.0
    OFFSET_X = 455
    OFFSET_Y = 260
    WIDTH = 420
    HEIGHT = 420
    
    root_dir = '/Users/JonathanLee/Desktop/sandbox/vision/'
    data_dir = root_dir + 'data/'
    datasets_dir = data_dir + 'datasets/'
    frames_dir = data_dir + 'record_frames/'
    videos_dir = data_dir + 'record_videos/'

    def __init__(self):
        self.test = False
        self.deploy = False
        self.learn = False
        self.model_path = ""        # path to network architecture prototxt
        self.weights_path = ""      # path to weights (should match model)
        self.show = False           # whether to show a preview window from bincam
        self.record = False         # whether to record frames with bincam
        self.scales = Options.scales
        self.translations = Options.translations
        self.drift = Options.drift
