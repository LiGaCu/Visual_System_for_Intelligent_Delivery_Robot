#include <sstream>
#include <iostream>
#include <fstream>

#include <opencv2/core/ocl.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "CNN_Yolo3.h"

using namespace std;
using namespace cv;
using namespace dnn;

int myselfyolo::yoloinit()
{
	// Open file with classes names.
	string names =  "coco.names";//"E:\\YOLO\\yolo-coco\\"
	ifstream ifs(names.c_str());
	if (!ifs.is_open())
		CV_Error(Error::StsError, "File " + names + " not found");
	string line;
	while (getline(ifs, line))
	{
		classes.push_back(line);
//		cout << line << "	";
	}

	// Load a model.
	string model =  "yolov3-tiny.weights";//"E:\\YOLO\\yolo-coco\\"
	string config =  "yolov3-tiny.cfg";//"E:\\YOLO\\yolo-coco\\"
	net = readNet(model, config);
	net.setPreferableTarget(DNN_TARGET_OPENCL_FP16);
	outNames = net.getUnconnectedOutLayersNames();

	//while (waitKey(1) < 0)
	//{

	//	// Put efficiency information.
	//	std::vector<double> layersTimes;
	//	double freq = getTickFrequency() / 1000;
	//	double t = net.getPerfProfile(layersTimes) / freq;
	//	std::string label = format("Inference time: %.2f ms", t);
	//	putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
	//	imshow("haha", frame);
	//}
}

void myselfyolo::GPU_Detect(void)
{
	if (!cv::ocl::haveOpenCL())
	{
		cout << "OpenCL is not available..." << endl;
	}

	cv::ocl::Context context;
	if (!context.create(cv::ocl::Device::TYPE_GPU))
	{
		cout << "Failed creating the context..." << endl;
	}

	cout << context.ndevices() << " GPU devices are detected." << endl; //This bit provides an overview of the OpenCL devices you have in your computer
	for (int i = 0; i < context.ndevices(); i++)
	{
		cv::ocl::Device device = context.device(i);
		cout << "name:              " << device.name() << endl;
		cout << "available:         " << device.available() << endl;
		cout << "imageSupport:      " << device.imageSupport() << endl;
		cout << "OpenCL_C_Version:  " << device.OpenCL_C_Version() << endl;
		cout << endl;
	}
}

void myselfyolo::yoloprocess(Mat& frame)
{
	blobFromImage(frame, blob, 1 / 255.0, Size(416, 416), true, false);
	net.setInput(blob);
	vector<Mat> outs;
	net.forward(outs, outNames);
	postprocess(frame, outs, net);
}

void myselfyolo::postprocess(Mat& frame, const std::vector<Mat>& outs, Net& net)
{
	static std::vector<int> outLayers = net.getUnconnectedOutLayers();
	static std::string outLayerType = net.getLayer(outLayers[0])->type;

	std::vector<int> classIds;
	std::vector<float> confidences;
	std::vector<Rect> boxes;
	if (outLayerType == "DetectionOutput")	//this part is useless when using Yolo
	{
		// Network produces output blob with a shape 1x1xNx7 where N is a number of
		// detections and an every detection is a vector of values
		// [batchId, classId, confidence, left, top, right, bottom]
		CV_Assert(outs.size() > 0);
		for (size_t k = 0; k < outs.size(); k++)
		{
			float* data = (float*)outs[k].data;
			for (size_t i = 0; i < outs[k].total(); i += 7)
			{
				float confidence = data[i + 2];
				if (confidence > 0.5)
				{
					int left = (int)data[i + 3];
					int top = (int)data[i + 4];
					int right = (int)data[i + 5];
					int bottom = (int)data[i + 6];
					int width = right - left + 1;
					int height = bottom - top + 1;
					if (width * height <= 1)
					{
						left = (int)(data[i + 3] * frame.cols);
						top = (int)(data[i + 4] * frame.rows);
						right = (int)(data[i + 5] * frame.cols);
						bottom = (int)(data[i + 6] * frame.rows);
						width = right - left + 1;
						height = bottom - top + 1;
					}
					classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
					boxes.push_back(Rect(left, top, width, height));
					confidences.push_back(confidence);
				}
			}
		}
	}
	else if (outLayerType == "Region")	// Yolo use this part
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
				if (confidence > 0.5)
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

	std::vector<int> indices;
	NMSBoxes(boxes, confidences, 0.5, 0.3, indices);
	for (size_t i = 0; i < indices.size(); ++i)
	{
		int idx = indices[i];
		Rect box = boxes[idx];
		drawPred(classIds[idx], confidences[idx], box.x, box.y,
			box.x + box.width, box.y + box.height, frame);
	}
}

void myselfyolo::drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
	rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

	std::string label = format("%.2f", conf);
	if (!classes.empty())
	{
		CV_Assert(classId < (int)classes.size());
		label = classes[classId] + ": " + label;
	}

	int baseLine;
	Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

	top = max(top, labelSize.height);
	rectangle(frame, Point(left, top - labelSize.height),
		Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
	putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}