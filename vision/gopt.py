"""
This class is intended to act as a blueprint for instances
of options for the gripper learning and deploying processes.
Here you can define states and properties of the gripper learner
as well as how data is managed to a certain degree.
Some API accept instances of options as arguments
"""

class RunOptions():

    def __init__(self):
        self.test = False
        self.deploy = False
        self.learn = False
        self.scales = None
        self.translations = None
        self.drift = None
