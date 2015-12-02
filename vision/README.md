# Learning in clutter

## Gripper controls

The files in this root directory are responsible for handling gripper controls
from user input and from neural net predictions.

#### Xbox controller with OS X Yosemite (10.10.x)

Install [Xbox controller driver](http://tattiebogle.net/index.php/ProjectRoot/Xbox360Controller/OsxDriver) version 0.11. Other drivers may interpret buttons on the Xbox controller differently.
Do not download the most recent version (0.12) as it is not compatible with Yosemite.
Follow driver instructions and restart machine.

Open system preferences and you should see an icon "Xbox 360 Controllers" in the bottom row.
Click that and plug in the controller to ensure it is recognized in "Device."

You can run test_joystick.py with `$ python test_joystick.py` and test the buttons and joysticks. All the code in this repo assumes that the driver
recognizes the d-pad as buttons 0-3 rather than as a "hat." If your buttons are interpreted as anything else, ensure you downloaded the correct driver.

Exit test_joystick.py and connect the arduinos. Check in the arduino IDE for the USB ports
and change them if necessary in IzzyRun.py and TurnTable.py.

Run IzzyRun.py with `$ python IzzyRun.py`

Most of the control code was written by Dave Gealy.

## Net

The Net module handles all the procedures and data necessary for training and deploying
the conv net.

If you've copied this module, you'll need to change the path in Net/constants/__init__.py
to point to your own Net directory.

## Vision pipeline

*calibrate* and *bincam* are the important modules in the pipeline. See README in pipeline
for instructions on *calibrate*. Calibration must occur before any video capture is used.

Create an instance of a **BinaryCamera** `bincam = BinaryCamera('meta.txt')` while supplying the appropriate path to the textfile metadata.
You should have obtained a *meta.txt* file from running *calibrate*.

Now you can call instance methods **read_frame** and **read_binary_frame** which reads a cropped raw image from the video and a cropped binary segmentation respectively.
Both methods have an optional "show" parameter which will open a window showing the images. The images from both methods are returned.
It should be noted that read_binary_frame is significantly slower than **read_frame** as it has to segment.

If speed is a concern, you may want to consider reading using **read_frame** and then applying **read_binary_frame** 
in post processing.
