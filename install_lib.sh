#!/bin/bash

mkdir "/home/pi/Documents/drone/binaries"
mkdir "/home/pi/Documents/drone/images"
 g++ -I/usr/local/include/opencv4 -I/usr/local/include/opencv4/opencv2 -L/usr/local/lib/ -g -o ./binaries/detectTiny  ./src/mappingTiny.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_imgcodecs -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_stitching -lopencv_dnn -lstdc++fs
cp -r ./dronescripts/* /usr/lib/python2.7/