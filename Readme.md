deps: 
needs opencv 3.1 or above

how to compile
g++ -std=gnu++11 -o projectAndCalc projectAndCalc.cpp `pkg-config opencv --cflags --libs`

this example is based on a really good discussed example
@see https://stackoverflow.com/questions/44104633/transforming-2d-image-coordinates-to-3d-world-coordinates-with-z-0
@see https://github.com/rodolfoap/OpenCV-2Dto3D
