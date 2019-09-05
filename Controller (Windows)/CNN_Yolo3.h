#ifndef CNN_YOLO3_H
#define CNN_YOLO3_H

#include <sstream>
#include <iostream>
#include <fstream>

#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;
using namespace dnn;

class myselfyolo
{
public:
	int yoloinit();
	void GPU_Detect();
	void yoloprocess(Mat& frame);
private:
	Mat blob;
	vector <string> classes;
	Net net;
	vector<String> outNames;
	void myselfyolo::postprocess(Mat& frame, const std::vector<Mat>& outs, Net& net);
	void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);
};
#endif