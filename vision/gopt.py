"""
GRIPPER OPTIONS
This class is intended to act as a blueprint for instances
of options for the gripper learning and deploying processes.
Pass to appropriate API

This is based on leveldb's options
"""

class GripperOptions():
    translations = [0.0, 0.0, 0.0, 0.0]
    scales = [40.0, 120.0, 90.0, 100.0]
    drift = 20.0

    def __init__(self):
        self.test = False
        self.deploy = False
        self.learn = False
        self.model_path = ""        # path to network architecture prototxt
        self.weights_path = ""      # path to weights (should match model)
        self.show = False           # whether to show a preview window from bincam
        self.record = False         # whether to record frames with bincam
        self.scales = GripperOptions.scales
        self.translations = GripperOptions.translations
        self.drift = GripperOptions.drift
