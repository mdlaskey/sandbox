# Vision Pipeline Segmentation

Before using bincam, you must calibrate the camera with the table.

Calibrate the video frame by executing `$ python calibrate.py`. In order to calibrate, you must place a RED dot (small but large enough to be picked up by the camera) in the center of the rotating table. Nothing should be visible in the cropped frame except for the green table background, the red dot, and the dark circle around the rotating table.

If necessary, edit **const.py** in the utilities directory to adjust the frame. The width and height of the frame should be correct for the distance between the table top and the ceiling (i.e. The dark ring nearly touches the edges of the frame).

The calibration will record 2 pieces of information. The coordinates of the red dot in the frame and the distance to the darkest point in the frame which should be on the circle. An image will appear showing what points were picked.

If the calibration chose the wrong points, ensure that the table is clear and the red dot is visible and run it again. The data are recorded in meta.txt
