#ifndef AUTO_VR_IMAGEPROCESS_H
#define AUTO_VR_IMAGEPROCESS_H

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp> 

#include<algorithm>

using namespace cv;
using namespace std;

class CVprocess
{
    public:

        CVprocess( VideoCapture cap, Size dsize );
        ~CVprocess();
        void processing(void);
        void confirmlineangle(void);

        Mat imageBGR;
        double angle=90;
        int lineLR[2][4];
        int havefoundL_flag = 0, havefoundR_flag = 0;
//        double globalspeed=1500;

    private:

        int floodfillprocess(Mat& processimage, Mat& grayforseed, Point& seedoutput, int lo, int up, Mat& outputmask);
        int floodfillprocess(Mat& processimage, Point seedinput, int lo, int up, Mat& outputmask);
        Point findwhitepoint(Mat& Inputimage);
        void drawseed(Mat& outputimage, Point seed);
        void linesort(Mat& Outputimage, vector<Vec4i>& lines);
        void roadiamgefilter(vector<Mat>& pastimage, Mat& newimage);

        Size dsize;
        Mat imageHSV;
        Mat mask1, mask2, mask3, mask4;
        Mat roadimage1, roadimage2;

        int b = 0, r = 0, g = 255;
        int loDiff1 = 5, upDiff1 = 5;
        int loDiff2 = 3, upDiff2 = 3;
        int loDiff3 = 50, upDiff3 = 50;
        int ffillMode = 2;
        int connectivity = 4;
        int isColor = true;
        int newMaskVal = 255;
        
        int loThreshold = 50, upThreshold = 70;

        Mat frame[3]={Mat(dsize.width,dsize.height,CV_8UC1),Mat(dsize.width,dsize.height,CV_8UC1),Mat(dsize.width,dsize.height,CV_8UC1)};
        vector<Mat> pastroadiamge;
        int framepointer=0;
        
        int areaofmask1=0, areaofmask2=0;
        Point seedpointer1, seedpointer2;
        vector<Vec4i> lines;
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        int lineNum_L, lineNum_R;
        int SteerError;//= Outputimage.cols- StreerCenter;
};

#endif
