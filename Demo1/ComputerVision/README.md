This folder contains the code for determining the angle of an aruco marker from the center of the camera. This folder contains 2 attempts at doing so. ArucoAngle.py which works the best will be used for demo 1. CameraCalibration.py and PoseEstimation.py were used in conjunction with eachother in an effort to find a better solution, but the calibration was never good enough to provide more accurate results.

README.txt: Describes purpose and organization of folder.

ArucoAngle.py: Uses the camera to detect the aruco marker, the angle away it is from the center in the x direction, and displays it onto the LCD screen (USED IN DEMO 1)

CameraCalibration.py: obtain camera matrix and distotrtion coefficients using open cv chess board method, needs to be run before PoseEstimation can work (NOT USED FOR DEMO 1)

PoseEstimation.py: attempt to complete demo one using aruco pose estimation function, did not work as well as ArucoAngle.py (NOT USED FOR DEMO 1)
