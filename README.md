# PT_codes
## Differents codes for our PT
For each code, if you want to use a video instead of a webcam, you simply have to replace the argument of the cv2.VideoCapture() function by the path of your video. For exemple :
- cap = cv2.VideoCapture(2)     allows to use the webcam 2
- cap = cv2.VideoCapture("/home/user/Video/example.mp4")      allows to use the video example.mp4

### color.py 
The code for the color recognition without the ROS Part.

### qrcal.py and scale.py
The codes for automatic (qrcal) and manual (scale) calibration. Must be used with ROS2 and allows to calibrate the camera to use the measurement codes : picker.py and shape.py

### picker.py and shape.py
The codes for automatic (shape) and manual (picker) calibration. Must be used with ROS2 and will run after the calibration is done.
