#what: 
OpenCV Example
Key-Question: Interested how pixel distance could be calculated in mm space ? 

I did before a [camera calibration 
via the opencv example](https://docs.opencv.org/2.4/_downloads/camera_calibration.cpp) (output:camera_param_d4_n500.xml) 
[HowTo step-by-step] (https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html?highlight=calibration7)

Afterwars I retried the inverse homography on one of the calibration pictures to get real world distance measurements.
The chessboard I used had 25mm x 25mm squares. We do have mean 
projection error of 0.10 mm for 28 checked distances, which the code shows.

There could might be a better option for the corner detection using lib: 
https://github.com/DIPlib/diplib

This example is based on a really good discussed example
https://stackoverflow.com/questions/44104633/transforming-2d-image-coordinates-to-3d-world-coordinates-with-z-0
https://github.com/rodolfoap/OpenCV-2Dto3D

#deps: 
needs opencv 3.1 or above

#how to compile:
g++ -std=gnu++11 -o projectAndCalc projectAndCalc.cpp `pkg-config opencv --cflags --libs`

#execute: 
./projectAndCalc
