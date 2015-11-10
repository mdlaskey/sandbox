# Vision Segmentation

Obtaining the segmented video stream is a 2-step process.

### 1.
Calibrate the video frame by executing `$ python calibrate.py`. In order to calibrate, you must place a RED dot (small but large enough to be picked up by the camera) in the center of the rotating table. Nothing should be visible in the frame except for the green table background, the red dot, and the dark circle around the rotating table.

The calibration will record 2 pieces of information. The coordinates of the red dot in the frame and the distance to the darkest point in the frame which should be on the circle. An image will appear showing what points were picked.

If the calibration chose the wrong points, ensure that the table is clear and the red dot is visible and run it again. The data are recorded in hyperparams.txt

### 2. 
After running the calibration, you can remove the red dot and run `$ python segment.py` to get the segmented video stream from the camera. If the frame is shifted or the camera is moved in anyway, you may have to calibrate again.

You can record or take snapshots of the video stream by uncommenting the appropriate lines in `segment.py`

The point of doing all of this is to identify and segment out the circle around the rotating table.
