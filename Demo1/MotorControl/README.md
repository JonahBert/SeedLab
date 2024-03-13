README.txt: This folder includes all of the code in order to get the robot to drive straight and turn a specified distance. 

Go_Straight_Distance.ino: This uses two controllers, one on the forward velocity and one on the rotational velocity. These are then integrated and controlled using a PI controller on the distance forward and angle in radians. We used this just for going straight.

Turn_and_Go.ino:This uses two controllers, one on the forward velocity and one on the rotational velocity. These are then integrated and controlled using a PI controller on the distance forward and angle in radians. This implments a 5 second timer between the rotation and going straight to ensure that turning has been completed in order to go straight.
