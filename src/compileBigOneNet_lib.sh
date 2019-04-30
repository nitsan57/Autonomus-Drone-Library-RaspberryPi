#!/bin/bash


 g++ -I/usr/local/include/opencv4 -I/usr/local/include/opencv4/opencv2 -L/usr/local/lib/ -g -o detectBigOneNet  mappingBigOneNet.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_imgcodecs -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_stitching -lopencv_dnn -lstdc++fs