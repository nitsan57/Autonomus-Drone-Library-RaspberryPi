//#include "stdafx.h"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp" 
#include "opencv2/stitching.hpp" 
#include <fstream>
#include <sstream>
#include <iostream>

struct DroneLocation {
	float yaw;
	float lon;
	float alt;
	float lat;
};

#pragma warning(disable : 4996)
#define _USE_MATH_DEFINES
#include <math.h>
//#define CFG_PATH_SPP "yolov3-spp.cfg"
//#define WGT_PATH_SPP "yolov3-spp.weights"
//#define CFG_PATH "yolov3.cfg"
//#define WGT_PATH "yolov3.weights"
//#define CFG_PATH "yolov3-tiny.cfg"
//#define WGT_PATH "yolov3-tiny.weights"
#define CFG_PATH "yolov3-spp.cfg"
#define WGT_PATH "yolov3-spp.weights"

#define NAMES "coco.names"
#define DARKNET_PATH
#include <experimental/filesystem>
#define CAM_WIDTH_ANGLE (M_PI * 62.2 / 180)
#define CAM_HIGHT_ANGLE (M_PI * 48.8 / 180)
namespace fs = std::experimental::filesystem;
using namespace cv;
using namespace dnn;
std::vector<std::string> classes;
float confThreshold = 0.01f;
float nmsThreshold = 0.4;
double scale = (double)1 / 255;
bool swapRB = true;
int inpWidth = 608;//416;//608;
int inpHeight = 608;//416;//608;


 //void postprocess(Mat& frame, const std::vector<Mat>& out, Net& net);

void postprocess2(Mat& frame, const std::vector<Mat>& out, const std::vector<Mat>& out2, Net& net, Net& net2, std::vector<int>& classes_histogram, DroneLocation& droneLoc, std::ofstream&);

void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame, DroneLocation& droneLoc, std::ofstream&);

void callback(int pos, void* userdata);

std::vector<String> getOutputsNames(const Net& net);

std::vector<String> getOutputsNames2(const Net& net);

std::pair<float, float> getPixelSize(float distance, float imageHeight, float imageWidth);

//std::pair<float, float> getObjectSize(float pixelSizeW, float pixelSizeH, float width, float height, std::vector<int>& classes_histogram);

std::pair<float, float> getPixelSize(float distance, float imageWidth, float imageHeight) {
	float h_pixel = (sin(CAM_HIGHT_ANGLE / 2) * 2 * distance) / cos(CAM_HIGHT_ANGLE / 2);
	float w_pixel = (sin(CAM_WIDTH_ANGLE / 2) * 2 * distance) / cos(CAM_WIDTH_ANGLE / 2);
	return std::pair<float, float>{ w_pixel / imageWidth, h_pixel / imageHeight };
}

std::pair<float, float> getObjectSize(float pixelSizeW, float pixelSizeH, float width, float height, float alt, float resW, float resH) {
	std::pair<float, float> pixels = getPixelSize(100 * alt, resW, resH); // example! nitsan arye RES
	return std::pair<float, float>{pixels.first * width, pixels.second * height };
}
// get gps location and meter distance and return new gps cords
std::pair<double, double> getLocationMeters(double locationLat, double locationLon, double locationAlt, double dNorth, double dEast) {
	float earthRadius = 6378137.0;
	double dlat = dNorth / earthRadius;
	double dlong = dEast / (earthRadius * cos(locationLat * M_PI / 180));
	double newLat = locationLat + (dlat * 180 / M_PI);
	double newLong = locationLon + (dlong * 180 / M_PI);
	return std::pair<double, double> {newLong, newLat};
}

Mat detect_objects_single_image(double locationLat, double locationLon, double locationAlt, Mat& image, std::vector<int>& classes_histogram, DroneLocation& droneLoc, std::ofstream& myfile) {
	// Load a model.
	Net net{};
	//Net net = readNetFromDarknet(CFG_PATH_SPP, WGT_PATH_SPP);
	//net.setPreferableBackend(DNN_BACKEND_OPENCV);
	//net.setPreferableTarget(DNN_TARGET_CPU);

	Net net2 = readNetFromDarknet(CFG_PATH, WGT_PATH);
	net2.setPreferableBackend(DNN_BACKEND_OPENCV);
	net2.setPreferableTarget(DNN_TARGET_CPU);



	// Process frames.

	Mat frame, blob, blob2;
	frame = Mat(image);

	if (frame.empty())
	{
		CV_Error(Error::StsError, "no image");
		waitKey();
		return Mat{};
	}
	const cv::Scalar mean = { 0,0,0 };
	// Create a 4D blob from a frame.
	Size inpSize(inpWidth > 0 ? inpWidth : frame.cols,
		inpHeight > 0 ? inpHeight : frame.rows);
	blobFromImage(frame, blob, scale, inpSize, mean, swapRB, false);
	blobFromImage(frame, blob2, scale, inpSize, mean, swapRB, false);

	// Run a model.
	//net.setInput(blob);
	net2.setInput(blob2);

	//if (net.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
	//{
	//	resize(frame, frame, inpSize, cv::INTER_AREA);
	//	Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, CV_32F); //1.6f
	//	net.setInput(imInfo, "im_info");
	//}

	std::vector<Mat> outs;
	//std::vector<cv::String> outNames = getOutputsNames(net);
	//net.forward(outs, outNames);

	if (net2.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
	{
		//resize(frame, frame, inpSize, cv::INTER_AREA);
		Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, CV_32F); //1.6f
		net2.setInput(imInfo, "im_info");
	}
	std::vector<Mat> outs2;
	net2.forward(outs2, getOutputsNames2(net2));



	postprocess2(frame, outs, outs2, net, net2, classes_histogram, droneLoc, myfile);
	return frame;
}

std::pair<float, float> rotateCords(float x, float y, float yaw) {
	float cos_yaw = cos(yaw);
	float sin_yaw = sin(yaw);
	float newX = cos_yaw * x - sin_yaw * y;
	float newY = sin_yaw * x + cos_yaw * y;
	return std::pair<float, float>{ newX, newY };
}

std::vector<float> parseName(std::string image_data) {
	int currentIndex = image_data.find(",");
	std::string str = image_data.substr(0, currentIndex);
	float lon = std::stof(str);
	image_data = image_data.erase(0, currentIndex + 1);
	currentIndex = image_data.find(",");
	str = image_data.substr(0, currentIndex);
	float lat = std::stof(str);
	image_data = image_data.erase(0, currentIndex + 1);
	currentIndex = image_data.find(",");
	str = image_data.substr(0, currentIndex);
	float alt = std::stof(str);
	image_data = image_data.erase(0, currentIndex + 1);
	currentIndex = image_data.find(".jpg");
	str = image_data.substr(0, currentIndex);
	float yaw = std::stof(str);
	std::vector<float> data;
	data.push_back(yaw);
	data.push_back(lon);
	data.push_back(alt);
	data.push_back(lat);

	return data;
}

int main(int argc, char** argv)
{
	std::cout << "here1" <<std::endl;

	std::string path = argv[1];
	// Define mode for stitching as panoroma  
	// (One out of manyf functions of Stitcher) 
	std::string file = NAMES;
	// Open file with classes names.

	std::ifstream ifs(file.c_str());
	if (!ifs.is_open())
		CV_Error(Error::StsError, "File " + file + " not found");
	std::string line;
	while (std::getline(ifs, line))
	{
		classes.push_back(line);
	}
	std::vector<int> classes_histogram(classes.size(), 0);

	std::cout << "here2" <<std::endl;
	std::vector <Mat> images;
	std::string path_to_dir = path;
	int i = 0;
	for (const auto & entry : fs::directory_iterator(path_to_dir)) {
		std::string im_path = entry.path().string();
		std::string image_data;
		int pos = im_path.find_last_of("/\\");
		image_data = im_path.substr(pos + 1, im_path.length() - pos);
		if (im_path.find(".jpg") != -1 && pos != -1 && image_data[0] != 'p') {
			//std::cout << im_path.substr(0, im_path.length() - 3) << std::endl;
			Mat image = cv::imread(im_path);
			std::ofstream myfile;
			myfile.open(im_path.substr(0, im_path.length() - 4) + ".txt");
			//myfile << "Writing this to a file.\n";
			std::cout << "here3" <<std::endl;
			Mat frame = {};
			cv::GaussianBlur(image, frame, cv::Size(9, 9), 0);
			cv::addWeighted(image, 1.15, frame, -0.15, 0, frame);
			image = frame;
			std::vector<float> data = parseName(image_data);
			DroneLocation droneLoc{ data.at(0), data.at(1), data.at(2), data.at(3) };
			Mat detection = detect_objects_single_image(0, 0, 0, image, classes_histogram, droneLoc, myfile);
			images.push_back(detection);
			imwrite(".\\images\\pred" + std::to_string(i) + ".jpg", detection);
			myfile.close();
			if (image.empty())
			{
				// Exit if image is not present 
				std::cout << "finished images already!" << i << std::endl;
				break;
			}
			std::cout << "here4" <<std::endl;
		}
		i++;

	}

	std::cout << "here5" <<std::endl;
	// Store a new image stiched from the given  
	//set of images as "result.jpg" 

	// Show the result 
	for (std::vector<int>::iterator it = classes_histogram.begin(); it != classes_histogram.end(); ++it) {
		if (*it != 0) {
			std::cout << "number of objcts" << std::endl;
			int classIndex = it - classes_histogram.begin();
			std::cout << classes[classIndex] << ": " << std::to_string(*it) << std::endl;
		}
	}


	return 0;
}


void postprocessHelper(Mat& frame, Net& net, const std::vector<Mat>& outs, std::vector<int>& outLayers, std::string& outLayerType, std::vector<Rect>& boxes, std::vector<float>& confidences, std::vector<int>& classIds) {

	if (net.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
	{
		// Network produces output blob with a shape 1x1xNx7 where N is a number of
		// detections and an every detection is a vector of values
		// [batchId, classId, confidence, left, top, right, bottom]
		CV_Assert(outs.size() == 1);
		float* data = (float*)outs[0].data;
		for (size_t i = 0; i < outs[0].total(); i += 7)
		{
			float confidence = data[i + 2];
			if (confidence > confThreshold)
			{
				int left = (int)data[i + 3];
				int top = (int)data[i + 4];
				int right = (int)data[i + 5];
				int bottom = (int)data[i + 6];
				int width = right - left + 1;
				int height = bottom - top + 1;
				classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
				boxes.push_back(Rect(left, top, width, height));
				confidences.push_back(confidence);
			}

		}
	}
	else if (outLayerType == "DetectionOutput")
	{
		// Network produces output blob with a shape 1x1xNx7 where N is a number of
		// detections and an every detection is a vector of values
		// [batchId, classId, confidence, left, top, right, bottom]
		CV_Assert(outs.size() == 1);
		float* data = (float*)outs[0].data;
		for (size_t i = 0; i < outs[0].total(); i += 7)
		{
			float confidence = data[i + 2];
			if (confidence > confThreshold)
			{
				int left = (int)(data[i + 3] * frame.cols);
				int top = (int)(data[i + 4] * frame.rows);
				int right = (int)(data[i + 5] * frame.cols);
				int bottom = (int)(data[i + 6] * frame.rows);
				int width = right - left + 1;
				int height = bottom - top + 1;
				classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
				boxes.push_back(Rect(left, top, width, height));
				confidences.push_back(confidence);
			}
		}
	}
	else if (outLayerType == "Region")
	{
		for (size_t i = 0; i < outs.size(); ++i)
		{
			// Network produces output blob with a shape NxC where N is a number of
			// detected objects and C is a number of classes + 4 where the first 4
			// numbers are [center_x, center_y, width, height]
			float* data = (float*)outs[i].data;
			for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
			{
				Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
				Point classIdPoint;
				double confidence;
				minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
				if (confidence > confThreshold)
				{
					int centerX = (int)(data[0] * frame.cols);
					int centerY = (int)(data[1] * frame.rows);
					int width = (int)(data[2] * frame.cols);
					int height = (int)(data[3] * frame.rows);
					int left = centerX - width / 2;
					int top = centerY - height / 2;

					classIds.push_back(classIdPoint.x);
					confidences.push_back((float)confidence);
					boxes.push_back(Rect(left, top, width, height));
				}
			}
		}
	}
	else
		CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);
}


void postprocess2(Mat& frame, const std::vector<Mat>& outs, const std::vector<Mat>& outs2, Net& net, Net& net2, std::vector<int>& classes_histogram, DroneLocation& droneLoc, std::ofstream& myfile)
{
	//static std::vector<int> outLayers = net.getUnconnectedOutLayers();
	static std::vector<int> outLayers2 = net2.getUnconnectedOutLayers();
	//static std::string outLayerType = net.getLayer(outLayers[0])->type;
	static std::string outLayerType2 = net2.getLayer(outLayers2[0])->type;
	std::vector<int> classIds;
	std::vector<float> confidences;
	std::vector<Rect> boxes;
	//postprocessHelper(frame, net, outs, outLayers, outLayerType, boxes, confidences, classIds);
	postprocessHelper(frame, net2, outs2, outLayers2, outLayerType2, boxes, confidences, classIds);
	std::vector<int> indices;
	NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
	for (size_t i = 0; i < indices.size(); ++i) {
		int idx = indices[i];
		Rect box = boxes[idx];
		drawPred(classIds[idx], confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, frame, droneLoc, myfile);
		classes_histogram[classIds[idx]]++;
	}
}


void writeToText(std::ofstream& myfile, cv::Mat& frame, int & objectX, int & objectY, std::string& label, std::pair<float, float>& objectSize, std::pair<float, float>& worldCords, int& left, int& top, int& right, int& bottom) {
	cv::Mat objectOnly = frame(cv::Rect(objectX, objectY, objectSize.first / 2, objectSize.second / 2));
	cv::Scalar AvgColors = cv::mean(objectOnly);
	int redAvgColor = AvgColors[0];
	int greenAvgColor = AvgColors[1];
	int blueAvgColor = AvgColors[2];
	myfile << label;
	myfile << ",";
	myfile << worldCords.first;
	myfile << "_";
	myfile << worldCords.second;
	myfile << ",";
	myfile << objectSize.first;
	myfile << "_";
	myfile << objectSize.second;
	myfile << ",";
	myfile << "(";
	myfile << redAvgColor;
	myfile << "_";
	myfile << greenAvgColor;
	myfile << "_";
	myfile << blueAvgColor;
	myfile << ")";
	myfile << ",";
	myfile << left;
	myfile << "_";
	myfile << top;
	myfile << "_";
	myfile << right;
	myfile << "_";
	myfile << bottom;
	myfile << "\n";
}

// gets gps cords and meter distance return new gps;
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame, DroneLocation& droneLoc, std::ofstream& myfile)
{
	float camera_centerX = frame.rows / 2;
	float camera_centerY = frame.cols / 2;
	rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));
	int objectY = (top + bottom) / 2;
	int objectX = (left + right) / 2;
	std::pair<float, float> metricDist = getObjectSize(0, 0, camera_centerX - objectX, camera_centerY - objectY, droneLoc.alt, frame.cols, frame.rows);
	metricDist.first /= 100;
	metricDist.second /= 100;
	std::pair<float, float> rotatedDist = rotateCords(metricDist.first, metricDist.second, droneLoc.yaw);
	std::pair<float, float> worldCords = getLocationMeters(droneLoc.lat, droneLoc.lon, droneLoc.alt, rotatedDist.first, rotatedDist.second);
	//std::string label = format("%.10f,%.10f", worldCords.first, worldCords.second);
	std::cout << "object lon: " << worldCords.first << ", object lat: " << worldCords.second << std::endl;
	std::string label;
	//nitsan !! arye example
	float numPixelsW = right - left;
	float numPixelsH = bottom - top;
	std::pair<float, float> objectsize = getObjectSize(numPixelsW, numPixelsH, numPixelsW, numPixelsH, droneLoc.alt, frame.cols, frame.rows);
	if (!classes.empty())
	{
		CV_Assert(classId < (int)classes.size());
		std::cout << "the " << classes[classId] << " on " << std::to_string(objectY) << " has the size: " << "w " << objectsize.first << " h: " << objectsize.second << std::endl;
		label = classes[classId];// +": " + label;

		writeToText(myfile, frame, objectY, objectY, label, objectsize, worldCords, left, top, right, bottom);

	}

	int baseLine;
	Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.8, 1, &baseLine);

	top = max(top, labelSize.height);
	rectangle(frame, Point(left, top - labelSize.height),
		Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
	putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(), 1);
}



void callback(int pos, void*)
{
	confThreshold = pos * 0.01f;
}

std::vector<String> getOutputsNames(const Net& net)
{
	static std::vector<String> names;
	if (names.empty())
	{
		std::vector<int> outLayers = net.getUnconnectedOutLayers();
		std::vector<String> layersNames = net.getLayerNames();
		names.resize(outLayers.size());
		for (size_t i = 0; i < outLayers.size(); ++i)
			names[i] = layersNames[outLayers[i] - 1];
	}
	return names;
}

std::vector<String> getOutputsNames2(const Net& net)
{
	static std::vector<String> names;
	if (names.empty())
	{
		std::vector<int> outLayers = net.getUnconnectedOutLayers();
		std::vector<String> layersNames = net.getLayerNames();
		names.resize(outLayers.size());
		for (size_t i = 0; i < outLayers.size(); ++i)
			names[i] = layersNames[outLayers[i] - 1];
	}
	return names;
}