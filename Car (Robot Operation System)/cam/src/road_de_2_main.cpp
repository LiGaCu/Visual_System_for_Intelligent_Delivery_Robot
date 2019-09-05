#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp> 

#include <string>
#include <sstream>
#include <iostream>

#include<algorithm>
#include<thread>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include "../include/keyinput.h"

using namespace cv;
using namespace std;

const char* keys =
{
	"{help h usage ? | | print this message}"
	"{@name | VideoTest | Video name, if not defined try to use VideoTest.avi}"
	"{source s | 0 | Video source, if not defined try to use cam0}"
};

int b = 0, r = 0, g = 255;
int loDiff1 = 5, upDiff1 = 5;
int loDiff2 = 3, upDiff2 = 3;
int loDiff3 = 50, upDiff3 = 50;
int ffillMode = 2;
int connectivity = 4;
int isColor = true;
int newMaskVal = 255;

int mode=1;
int runstill = 1;
double globalangle=90;
double globalspeed=1500;

int loThreshold = 50, upThreshold = 70;

Size dsize = Size(640, 360);

#define StreerCenter 840

void scan();
int floodfillprocess(Mat& processimage, Mat& grayforseed, Point& seedoutput, int lo, int up, Mat& outputmask);
int floodfillprocess(Mat& processimage, Point seedinput, int lo, int up, Mat& outputmask);
Point findwhitepoint(Mat& Inputimage);
void drawseed(Mat& outputimage, Point seed);
void linesort(Mat& Outputimage, vector<Vec4i>& lines);
void roadiamgefilter(vector<Mat>& pastimage, Mat& newimage);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "road_de_2");
	ros::NodeHandle n;
	ros::Publisher pub_= n.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);
	geometry_msgs::Twist SpeedAndAngle;

	cv::CommandLineParser parser(argc, argv, keys);
	parser.about("No showing Detection!");
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}

	Mat imageHSV, imageBGR;
	Mat mask1, mask2, mask3;
	Mat roadimage1, roadimage2;

	VideoCapture cap;
	int realtime_frame = 0;
	cap.open(0);//RoadVideo.mp4");//
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);  //设置捕获视频的宽度
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);  //设置捕获视频的高度

	char textshow[50];

	cap >> imageBGR;                                //从视频捕获一帧图像
	resize(imageBGR, imageBGR, dsize);
	mask1.create(imageBGR.rows + 2, imageBGR.cols + 2, CV_8UC1);
	mask2.create(imageBGR.rows + 2, imageBGR.cols + 2, CV_8UC1);
	mask3.create(imageBGR.rows + 2, imageBGR.cols + 2, CV_8UC1);
	Mat mask4;
	Mat frame[] = { Mat::zeros(imageBGR.size(),CV_8UC1),Mat::zeros(imageBGR.size(),CV_8UC1) ,Mat::zeros(imageBGR.size(),CV_8UC1) };
	vector<Mat> pastroadiamge(3, Mat::zeros(imageBGR.size(), CV_8UC1));

	int areaofmask1=0, areaofmask2=0;
	Point seedpointer1, seedpointer2;
	vector<Vec4i> lines;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

    thread keyboard(scan);
	keyboard.detach();

	while (runstill)
	{		
		cap >> imageBGR;                            //从相机捕获一帧图像
		resize(imageBGR, imageBGR, dsize, 0, 0, INTER_NEAREST);          //对捕捉的图像进行缩放操作

		cvtColor(imageBGR, imageHSV, COLOR_BGR2HSV);
		split(imageHSV, frame);
		mask1 = Scalar::all(0);
		mask2 = Scalar::all(0);
		mask3 = Scalar::all(0);
/*		areaofmask1 = floodfillprocess(frame[1], frame[1], seedpointer1, loDiff1, upDiff1, mask1);
		//		morphologyEx(mask1, mask1, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(5, 5)));
		roadimage1 = Mat(mask1, Rect(1, 1, imageBGR.cols, imageBGR.rows));
*/
		GaussianBlur(imageBGR, imageBGR, Size(13, 13), 0, 0);
		cvtColor(imageBGR, frame[2], COLOR_BGR2GRAY);
		areaofmask2 = floodfillprocess(imageBGR, frame[2], seedpointer2, loDiff2, upDiff2, mask2);
		roadimage2 = Mat(mask2, Rect(1, 1, imageBGR.cols, imageBGR.rows));

		drawseed(imageBGR, seedpointer2);
		//		inRange(imageBGR,Scalar(imageBGR.at<Vec3b>(seedlast.y, seedlast.x))- Scalar(loDiff3, loDiff3, loDiff3), Scalar(imageBGR.at<Vec3b>(seedlast.y, seedlast.x)) + Scalar(upDiff3, upDiff3, upDiff3),mask3);

		if (areaofmask1 > (imageBGR.cols*imageBGR.rows*0.3) && areaofmask2 > (imageBGR.cols*imageBGR.rows*0.3))
			bitwise_and(roadimage1, roadimage2, roadimage2);
		else if (areaofmask1 > (imageBGR.cols*imageBGR.rows*0.3) && areaofmask2 <= (imageBGR.cols*imageBGR.rows*0.3))
		{
			roadimage2 = roadimage1;
			areaofmask2 = areaofmask1;
		}

		//			morphologyEx(mask1, mask1, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(9, 9)));

		findContours(roadimage2, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		drawContours(roadimage2, contours, -1, Scalar(255, 255, 255),
			FILLED, LINE_8, hierarchy, 1);

		seedpointer2 = findwhitepoint(roadimage2);
		areaofmask2 = floodfillprocess(roadimage2, seedpointer2, loDiff2, upDiff2, mask3);
		roadimage2 = Mat(mask3, Rect(1, 1, imageBGR.cols, imageBGR.rows));

		mask4 = Mat(roadimage2,Rect(0,0, imageBGR.cols,imageBGR.rows/3));
		mask4 = Scalar::all(0);

		roadiamgefilter(pastroadiamge, roadimage2);
		threshold(roadimage2, roadimage2, 255 * 0.5, 255, THRESH_BINARY);
		frame[2].copyTo(roadimage2, roadimage2);

		Canny(roadimage2, roadimage2, loThreshold, upThreshold, 3); //对图像进行Canny算子的图像边缘检测

		HoughLinesP(roadimage2, lines, 1, CV_PI / 180, 50, 50, 30);
		/************************************************************************************************
		for (size_t i = 0; i < lines.size(); i++)
			line(imageBGR, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(255, 0, 255), 1, CV_AA);//最后一个参数4、8、CV_AA

		sprintf(textshow, "Frame : %d", realtime_frame);
		putText(imageBGR, textshow, Point(5, imageBGR.rows - 5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1, LINE_8, 0);
                ************************************************************************************************/
		linesort(imageBGR, lines);
		SpeedAndAngle.linear.x=globalspeed;
		SpeedAndAngle.angular.z=globalangle;
		pub_.publish(SpeedAndAngle); 
		ros::spinOnce();
	}
	SpeedAndAngle.linear.x=1500;
	SpeedAndAngle.angular.z=90;
	pub_.publish(SpeedAndAngle); 
	ros::spinOnce();
	
	mask1.release();
	mask2.release();
	mask3.release();
	mask4.release();
	imageHSV.release();
	imageBGR.release();
	roadimage1.release();
	roadimage2.release();
	frame[0].release();
	frame[1].release();
	frame[2].release();
	pastroadiamge[0].release();
	pastroadiamge[1].release();
	pastroadiamge[2].release();
	cap.release();
	return 0;
}

void scan()
{
	char keychar;
	while(runstill)
	{
		//cout<<"Press 's' to stop!\n";
		keychar=get1char();
		switch (keychar)
		{
			case 27:
				runstill=0;
				break;
			case ' ':
				if(mode==0)
				{
					mode=1;cout<<"Autonous Mode Setted!\n";
				}
				else
					mode =0;cout<<"Remote Mode Setted!\n";
				break;
			case '4':
				globalangle-=2;
				if(globalangle<45)
					globalangle=45;
				cout<<"Angle = "<<globalangle<<'\n';
				break;
			case '8':
				globalspeed+=20;
				if(globalspeed>1700)
					globalspeed=1700;
				cout<<"Speed = "<<globalspeed<<'\n';
				break;
			case '6':
				globalangle+=2;
				if(globalangle>135)
					globalangle=135;
				cout<<"Angle = "<<globalangle<<'\n';
				break;
			case '5':
				globalspeed-=20;
				if(globalspeed<1300)
					globalspeed=1300;
				cout<<"Speed = "<<globalspeed<<'\n';
				break;
			default:
				break;
		}
		int sleeptime=50000;
		do{
			sleeptime=usleep(sleeptime);
		}while(sleeptime>0);
	}
}

#define window_start_x  3 / 8
#define window_start_y  7 / 8
#define window_width  1 / 4
#define window_height  1 / 8

int floodfillprocess(Mat& processimage, Mat& grayforseed, Point& seedoutput, int lo, int up, Mat& outputmask)
{
	int flags = connectivity + (newMaskVal << 8) +
		(0 == 1 ? FLOODFILL_FIXED_RANGE : 0) + FLOODFILL_MASK_ONLY;
	Rect ccomp;
	int area;
	//blur(imagedst, imagedst, Size(7, 7), Point(-1,-1));
	//GaussianBlur(featureimage, featureimage, Size(13, 13), 0, 0);
	//cvtColor(colorimage, grayimage, COLOR_BGR2GRAY);//将双目源图像（彩色图）转为灰度数据
	//if (!isColor)
	//	colorimage = grayimage;
	Mat SeedField = grayforseed.clone();
	SeedField = Mat(SeedField, Rect(SeedField.cols *window_start_x, SeedField.rows *window_start_y, SeedField.cols *window_width, SeedField.rows *window_height));
	resize(SeedField, SeedField, Size(SeedField.cols / 4, SeedField.rows / 4), 0, 0, INTER_NEAREST);

	uchar *data = SeedField.data;
	Point seed;

	vector<int> datacounter; vector<int> datavalue;
	datavalue.push_back(*data);
	datacounter.push_back(1);
	for (int h = 0; h < SeedField.rows; ++h)
	{
		for (int w = 1; w < SeedField.cols; ++w)
		{
			data++;
			if (find(datavalue.begin(), datavalue.end(), *data) != datavalue.end())
			{
				datacounter[find(datavalue.begin(), datavalue.end(), *data) - datavalue.begin()]++;
			}
			else
			{
				datavalue.push_back(*data);
				datacounter.push_back(1);
			};
		}
	}
	vector<int>::iterator most = max_element(datacounter.begin(), datacounter.end());
	int mostposition = most - datacounter.begin();
	for (int h = SeedField.rows - 1; h >= 0 && seed.x == 0 && seed.y == 0; --h)
	{
		for (int w = SeedField.cols - 1; w >= 0 && seed.x == 0 && seed.y == 0; --w)
		{
			if (*data == datavalue[mostposition])
			{
				seed.x = w * 4 + grayforseed.cols * window_start_x;
				seed.y = h * 4 + grayforseed.rows * window_start_y;
			}
			if (h != 0 || w != 0) data--;
		}
	}
	seedoutput = seed;
	Scalar newVal = isColor ? Scalar(b, g, r) : Scalar(r*0.299 + g*0.587 + b*0.114);
	area = floodFill(processimage, outputmask, seed, newVal, &ccomp, Scalar(lo, lo, lo),
		Scalar(up, up, up), flags);

	//rectangle(imagedst, Rect(imagedst.cols *3/ 8, imagedst.rows * window_start_y, imagedst.cols window_width, imagedst.rows *window_height), Scalar(255, 255, 255), 2, 8);

	SeedField.release();
	return area;
}

int floodfillprocess(Mat& processimage, Point seedinput, int lo, int up, Mat& outputmask)
{
	int flags = connectivity + (newMaskVal << 8) +
		(0 == 1 ? FLOODFILL_FIXED_RANGE : 0) + FLOODFILL_MASK_ONLY;
	Rect ccomp;
	int area;

	Scalar newVal = isColor ? Scalar(b, g, r) : Scalar(r*0.299 + g*0.587 + b*0.114);
	area = floodFill(processimage, outputmask, seedinput, newVal, &ccomp, Scalar(lo, lo, lo),
		Scalar(up, up, up), flags);
	return area;
}

Point findwhitepoint(Mat& Inputimage)
{
	Mat SeedField = Inputimage.clone();
	SeedField = Mat(SeedField, Rect(SeedField.cols *window_start_x, SeedField.rows *window_start_y, SeedField.cols *window_width, SeedField.rows *window_height));
	resize(SeedField, SeedField, Size(SeedField.cols / 4, SeedField.rows / 4), 0, 0, INTER_NEAREST);

	uchar *data = SeedField.data;
	Point seed;

	for (int h = 0; h < SeedField.rows && seed.x == 0 && seed.y == 0; ++h)
	{
		for (int w = 0; w <SeedField.cols && seed.x == 0 && seed.y == 0; ++w)
		{
			if (*data == 255)
			{
				seed.x = w * 4 + Inputimage.cols * window_start_x;
				seed.y = h * 4 + Inputimage.rows * window_start_y;
			}
			data++;
		}
	}
	SeedField.release();
	return seed;
}

void drawseed(Mat& outputimage, Point seed)
{
	int numrows, numcols;
	numrows = outputimage.rows*window_height / 4;
	numcols = outputimage.cols*window_width / 4;
	for (int h = 0; h < numrows; ++h)
	{
		for (int w = 0; w < numcols; ++w)
		{
			if (Point(w * 4 + outputimage.cols * window_start_x, h * 4 + outputimage.rows * window_start_y) == seed)
				circle(outputimage, Point(w * 4 + outputimage.cols * window_start_x, h * 4 + outputimage.rows * window_start_y), 7, Scalar(255, 0, 255), -1);
			else circle(outputimage, Point(w * 4 + outputimage.cols * window_start_x, h * 4 + outputimage.rows * window_start_y), 1, Scalar(255, 255, 255), -1);
		}
	}
}

int framepointer;
void roadiamgefilter(vector<Mat>& pastimage, Mat& newimage)
{
	int a, b, c;
	Mat temp;
	if (framepointer == pastimage.size())
		framepointer = 0;
	temp = newimage.clone();
	pastimage.erase(pastimage.begin() + framepointer, pastimage.begin() + framepointer + 1);
	pastimage.insert(pastimage.begin() + framepointer, temp);

	a = framepointer;
	b = (framepointer>0) ? (framepointer - 1) : (framepointer + 2);
	c = (framepointer>1) ? (framepointer - 2) : (framepointer + 1);

	newimage = 0.34*pastimage[a] + 0.33*pastimage[b] + 0.33*pastimage[c];//0.35,0.3,0.2,0.15	+ 0*pastimage[d];
	framepointer++;
	temp.release();
}

void linesort(Mat& Outputimage, vector<Vec4i>& lines)
{
	vector<long int>  lengthtimeilength;
	int lineNum_L, lineNum_R;
	int havefoundL_flag = 0, havefoundR_flag = 0;
	double ellipse_a_L=100, ellipse_a_R=100;

	vector<float> slope;
	float slope_limit_perameter;
	vector<int> point1_L, point2_L;
	/*---------------坐标系平移---------------------------------------*/
	for (size_t i = 0; i < lines.size(); i++)
	{
		lines[i][0] -= Outputimage.cols / 2;
		lines[i][1] -= Outputimage.rows;
		lines[i][2] -= Outputimage.cols / 2;
		lines[i][3] -= Outputimage.rows;
	}
	/*------------------------检测与椭圆相交时最小椭圆----------------------*/
	float ellipse_b_to_a = 0.7;
	double ellipse_a; //ellipse_b_to_a小于1时是扁的椭圆
	vector<double> ellipse_a_s;
	double AtimesA, BtimesB, CtimesC;
	float crosspoint_x;
	vector<float> crosspoint_x_s;
	for (size_t i = 0; i < lines.size(); i++)
	{
		lengthtimeilength.push_back((lines[i][3] - lines[i][1])*(lines[i][3] - lines[i][1]) + (lines[i][2] - lines[i][0])*(lines[i][2] - lines[i][0]));
		if ((lines[i][2] - lines[i][0]) != 0)
			slope.push_back(((float)(lines[i][3] - lines[i][1]) / (float)(lines[i][2] - lines[i][0])));
		else
			slope.push_back(99999);
		point1_L.push_back(((lines[i][0] - Outputimage.cols / 2) > 0) ? 0 : 1);
		point2_L.push_back(((lines[i][2] - Outputimage.cols / 2) > 0) ? 0 : 1);
		AtimesA = lines[i][1] - lines[i][3];
		BtimesB = lines[i][2] - lines[i][0];
		CtimesC = lines[i][0] * lines[i][3] - lines[i][2] * lines[i][1];
		if (AtimesA != 0)
		{
			BtimesB = BtimesB / AtimesA;
			BtimesB *= BtimesB;
			CtimesC = CtimesC / AtimesA;
			CtimesC *= CtimesC;
			AtimesA = 1;
			ellipse_a = (CtimesC / AtimesA);
			ellipse_a -= (BtimesB*CtimesC*ellipse_b_to_a / (ellipse_b_to_a*BtimesB*AtimesA + AtimesA*AtimesA));
			crosspoint_x = (float)(-1 * (lines[i][0] * lines[i][3] - lines[i][2] * lines[i][1]) / (lines[i][1] - lines[i][3]) / (AtimesA + ellipse_b_to_a*BtimesB));
			//注意计算时应用的参数都是归一化后的
		}
		else                //A=0时
		{
			CtimesC = CtimesC / BtimesB;
			CtimesC *= CtimesC;
			BtimesB = 1;
			ellipse_a = (CtimesC / (ellipse_b_to_a*BtimesB));
			ellipse_a -= (AtimesA*CtimesC / (ellipse_b_to_a*AtimesA*BtimesB + ellipse_b_to_a*ellipse_b_to_a*BtimesB*BtimesB));
			crosspoint_x = 0;
		}
		if ((lines[i][0] >= crosspoint_x && lines[i][2] <= crosspoint_x) || (lines[i][0] <= crosspoint_x && lines[i][2] >= crosspoint_x))
			ellipse_a_s.push_back(ellipse_a);
		else
		{
			if (abs(lines[i][0] - crosspoint_x) < abs(lines[i][2] - crosspoint_x))
			{
				ellipse_a_s.push_back((lines[i][0] * lines[i][0] + lines[i][1] * lines[i][1] / ellipse_b_to_a));
				crosspoint_x = (float)(lines[i][0]);
			}
			else
			{
				ellipse_a_s.push_back((lines[i][2] * lines[i][2] + lines[i][3] * lines[i][3] / ellipse_b_to_a));
				crosspoint_x = (float)(lines[i][2]);
			}
		}
		crosspoint_x_s.push_back(crosspoint_x);
	}
	/*------------------------找出左右边界-----------------------*/

	//先将所有数组从新排序,
	double tempnum1;
	Vec4i tempnum2;
	float tempnum3;
	int tempnum4;
	long int tempnum5;

	for (size_t i = 0; i < lines.size(); i++)
	{
		size_t testposition = i;
		double temp;
		temp = ellipse_a_s[i];
		for (size_t t = i; t < lines.size(); t++)
		{
			if (ellipse_a_s[t] < temp)
			{
				testposition = t;
				temp = ellipse_a_s[t];
			}
		}
		tempnum1 = ellipse_a_s[testposition];
		ellipse_a_s[testposition] = ellipse_a_s[i];
		ellipse_a_s[i] = tempnum1;
		tempnum2 = lines[testposition];
		lines[testposition] = lines[i];
		lines[i] = tempnum2;
		tempnum3 = slope[testposition];
		slope[testposition] = slope[i];
		slope[i] = tempnum3;
		tempnum4 = point1_L[testposition];
		point1_L[testposition] = point1_L[i];
		point1_L[i] = tempnum4;
		tempnum4 = point2_L[testposition];
		point2_L[testposition] = point2_L[i];
		point2_L[i] = tempnum4;
		tempnum5 = lengthtimeilength[testposition];
		lengthtimeilength[testposition] = lengthtimeilength[i];
		lengthtimeilength[i] = tempnum5;
		tempnum3 = crosspoint_x_s[testposition];
		crosspoint_x_s[testposition] = crosspoint_x_s[i];
		crosspoint_x_s[i] = tempnum3;
	}
	//坐标系反变换
	for (size_t i = 0; i < lines.size(); i++)
	{
		lines[i][0] += Outputimage.cols / 2;
		lines[i][1] += Outputimage.rows;
		lines[i][2] += Outputimage.cols / 2;
		lines[i][3] += Outputimage.rows;
		crosspoint_x_s[i] += Outputimage.cols / 2;
	}

	//开始顺序筛选
	for (size_t i = 0; i < lines.size(); i++)
	{
		slope_limit_perameter = (float)(10000 / ellipse_a_s[i]);
		if (!havefoundL_flag &&  ellipse_a_s[i]>100 * 100 && slope[i]< -slope_limit_perameter + 0.05 && crosspoint_x_s[i]<Outputimage.cols / 2)
		{
			havefoundL_flag = 1;
			lineNum_L = i;
			ellipse_a_L=ellipse_a_s[i];
		}

		if (!havefoundR_flag &&  ellipse_a_s[i] > 100 * 100 && slope[i] > slope_limit_perameter - 0.05 && slope[i] != 99999 && crosspoint_x_s[i]>Outputimage.cols / 2)
		{
			havefoundR_flag = 1;
			lineNum_R = i;
			ellipse_a_R = ellipse_a_s[i];
		}
	}

	/*---------先检测是否有线段会相切，并减小扫描半径-----------*/

	/*------------------检测只能相交、不会相切的圆------------*/

	/*------------------------绘制图像-------------------------*/
/*
	if (havefoundL_flag)
	{
		line(Outputimage, Point(lines[lineNum_L][0], lines[lineNum_L][1]), Point(lines[lineNum_L][2], lines[lineNum_L][3]), Scalar(0, 255, 0), 3, CV_AA);
		ellipse(Outputimage, Point(Outputimage.cols / 2, Outputimage.rows),Size((int)(sqrt(ellipse_a_L)), (int)(sqrt(ellipse_a_L*ellipse_b_to_a))),
			0.0, 181, 359, Scalar(255, 200, 200), 1, 8, 0);
	}
	if (havefoundR_flag)
	{
		line(Outputimage, Point(lines[lineNum_R][0], lines[lineNum_R][1]), Point(lines[lineNum_R][2], lines[lineNum_R][3]), Scalar(0, 255, 0), 3, CV_AA);
		ellipse(Outputimage, Point(Outputimage.cols / 2, Outputimage.rows),Size((int)(sqrt(ellipse_a_R)), (int)(sqrt(ellipse_a_R*ellipse_b_to_a))),
			0.0, 181, 359, Scalar(200, 200, 255), 1, 8, 0);
	}*/



	/*
	int end_1, end_2, end_3, end_4;
	vector<Point> validpoint;

	if (havefoundL_flag)
	{
	end_1 = (0 - lines[lineNum_L][3])*(lines[lineNum_L][0] - lines[lineNum_L][2]) / (lines[lineNum_L][1] - lines[lineNum_L][3]) + lines[lineNum_L][2];
	end_2 = (Outputimage.cols - 1 - lines[lineNum_L][2])*(lines[lineNum_L][1] - lines[lineNum_L][3]) / (lines[lineNum_L][0] - lines[lineNum_L][2]) + lines[lineNum_L][3];
	end_3 = (Outputimage.rows - 1 - lines[lineNum_L][3])*(lines[lineNum_L][0] - lines[lineNum_L][2]) / (lines[lineNum_L][1] - lines[lineNum_L][3]) + lines[lineNum_L][2];
	end_4 = (0 - lines[lineNum_L][2])*(lines[lineNum_L][1] - lines[lineNum_L][3]) / (lines[lineNum_L][0] - lines[lineNum_L][2]) + lines[lineNum_L][3];
	for (int i = 0; i < 2; )
	{
	if (end_1 >= 0 && end_1 < Outputimage.cols)
	{
	validpoint.push_back(Point(end_1, 0));
	i++;
	end_1 = -1;
	}
	else if (end_2 >= 0 && end_2 < Outputimage.rows)
	{
	validpoint.push_back(Point(Outputimage.cols - 1, end_2));
	i++;
	end_2 = -1;
	}
	else if (end_3 >= 0 && end_3 < Outputimage.cols)
	{
	validpoint.push_back(Point(end_3, Outputimage.rows - 1));
	i++;
	end_3 = -1;
	}
	else if (end_4 >= 0 && end_4 < Outputimage.rows)
	{
	validpoint.push_back(Point(0, end_4));
	i++;
	end_4 = -1;
	}
	}
	line(Outputimage, validpoint[0], validpoint[1], Scalar(0, 255, 0), 3, CV_AA);
	}
	if (havefoundR_flag)
	{
	validpoint.clear();
	end_1 = (0 - lines[lineNum_R][3])*(lines[lineNum_R][0] - lines[lineNum_R][2]) / (lines[lineNum_R][1] - lines[lineNum_R][3]) + lines[lineNum_R][2];
	end_2 = (Outputimage.cols - 1 - lines[lineNum_R][2])*(lines[lineNum_R][1] - lines[lineNum_R][3]) / (lines[lineNum_R][0] - lines[lineNum_R][2]) + lines[lineNum_R][3];
	end_3 = (Outputimage.rows - 1 - lines[lineNum_R][3])*(lines[lineNum_R][0] - lines[lineNum_R][2]) / (lines[lineNum_R][1] - lines[lineNum_R][3]) + lines[lineNum_R][2];
	end_4 = (0 - lines[lineNum_R][2])*(lines[lineNum_R][1] - lines[lineNum_R][3]) / (lines[lineNum_R][0] - lines[lineNum_R][2]) + lines[lineNum_R][3];
	for (int i = 0; i < 2; )
	{
	if (end_1 >= 0 && end_1 < Outputimage.cols)
	{
	validpoint.push_back(Point(end_1, 0));
	i++;
	end_1 = -1;
	}
	else if (end_2 >= 0 && end_2 < Outputimage.rows)
	{
	validpoint.push_back(Point(Outputimage.cols - 1, end_2));
	i++;
	end_2 = -1;
	}
	else if (end_3 >= 0 && end_3 < Outputimage.cols)
	{
	validpoint.push_back(Point(end_3, Outputimage.rows - 1));
	i++;
	end_3 = -1;
	}
	else if (end_4 >= 0 && end_4 < Outputimage.rows)
	{
	validpoint.push_back(Point(0, end_4));
	i++;
	end_4 = -1;
	}
	}
	line(Outputimage, validpoint[0], validpoint[1], Scalar(0, 255, 0), 3, CV_AA);
	}
	*/


	int SteerError;//= Outputimage.cols- StreerCenter;
	if (havefoundR_flag)
	{
		SteerError = (Outputimage.rows - lines[lineNum_R][1]) / slope[lineNum_R] + lines[lineNum_R][0];
		SteerError = SteerError - StreerCenter;
	}
	else
		SteerError = 300;
	//char textshow[50];
	//if(SteerError>0)
	//	sprintf(textshow, "Turn to : R  %04.1f'", ((float)(SteerError))/10);
	//else
	//	sprintf(textshow, "Turn to : L  %04.1f'", (abs((float)(SteerError))/10));
	//putText(Outputimage, textshow, Point(Outputimage.cols / 2 - 150, 40), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 100), 2, LINE_8, 0);

	double angle=90-((float)(SteerError))/10;

	if(mode==1)
	{
		globalangle=angle;
	}

}
