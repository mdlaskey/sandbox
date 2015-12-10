"""
GRIPPER OPTIONS
This class is intended to act as a blueprint for instances
of options for the gripper learning and deploying processes.
Here you can define states and properties of the gripper learner
as well as how data is managed to a certain degree.
Some API accept instances of options as arguments
"""

class GripperOptions():

    def __init__(self):
        self.test = False
        self.deploy = False
        self.learn = False
        self.scales = None          # scale controls
        self.translations = None    # translate controls
        self.drift = None           # error in controls
        self.model_path = ""        # path to network architecture prototxt
        self.weights_path = ""      # path to weights (should match model)
        self.show = False           # whether to show a preview window from bincam
