what: 

Interest how pixel distance could be calculated in mm space ? 

I did before a camera calibration via opencv example (output:camera_param_d4_n500.xml) 
and retried the inverse homography on one of the calibration pictures.
The chessboard has 25mm x 25mm squares. We do have mean 
projection error of 0.10 mm for 28 checked distances.

There could might be a better option for the corner detection using 

This example is based on a really good discussed example
@see https://stackoverflow.com/questions/44104633/transforming-2d-image-coordinates-to-3d-world-coordinates-with-z-0
@see https://github.com/rodolfoap/OpenCV-2Dto3D

deps: 
needs opencv 3.1 or above

how to compile:
g++ -std=gnu++11 -o projectAndCalc projectAndCalc.cpp `pkg-config opencv --cflags --libs`

execute: 
./projectAndCalc



