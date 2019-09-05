//程序版本：2019-3-16-V1.0


#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>    
#include <opencv2/opencv.hpp> 
#include <thread>

using namespace std;
using namespace cv;

const char* keys =
{
	"{help h usage ? | | print this message}"
	"{@name | VideoTest | Video name, if not defined try to use VideoTest.avi}"
	"{source s | 0 | Video source, if not defined try to use cam0}"
};

int runstill=1;
void scan()
{
	char keychar;
	while(runstill)
	{
		cout<<"Press 's' to stop!\n";
		cin>>keychar;
		if(keychar=='s')
			runstill=0;
		//sleep(0.03);
	}
}

int main(int argc, const char** argv)            //程序主函数
{
	CommandLineParser parser(argc, argv, keys);
	parser.about("Video Capture");

	if (parser.has("help"))                      //帮助信息
	{
		parser.printMessage();
		return 0;
	}

	if (!parser.check())
	{
		parser.printErrors();
		return 0;
	}

	VideoCapture cap;
	string videoname = parser.get<string>(0);
	videoname=videoname+".avi";
	int videosource = parser.get<int>("source");
	
	if(videosource==0||videosource==2)
	{
		cap.open(videosource);
		cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);  //设置捕获视频的宽度
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);  //设置捕获视频的高度
	}
	else
	{
		cout<<"Invilid Camera!\n";
		return -1;
	}

	Mat frame;
	cap >> frame;                                //从相机捕获一帧图像
	cout<<"Setted Size:"<<frame.cols<<"*"<<frame.rows<<"\n";
	cout<<"Camera rate:"<<cap.get(CV_CAP_PROP_FPS)<<"\n";

	//double fScale = 1.0 ;                         //定义缩放系数，对2560*720图像进行缩放显示（2560*720图像过大，需要缩放才可完整显示在屏幕）  
	//Size dsize = Size(frame.cols*fScale, frame.rows*fScale);
	//Mat imagedst = Mat(dsize, CV_32S);
	//resize(frame, imagedst, dsize);

	//--此处需要根据视频显示的实际速度来设置AVI视频的帧率
	//--视频文件的帧率值rate，要和抓取图像的频率相对0应。比如：程序先抓图，再进行一次人脸识别耗时0.05秒，再抓图，再做人脸识别，则实际抓图为20次/秒，帧率也就是20
	double rate = 26.0;                          //视频的帧率  
	//--------------------------------------------------------------------------------------

	Size videoSize(1280, 720);                   //定义视频文件的分辨率
	const char* name=videoname.data();
	VideoWriter writer(name, CV_FOURCC('P', 'I', 'M', '1'), rate, videoSize);
	
	thread keyboard(scan);
	keyboard.detach();

	while (runstill)
	{
		cap >> frame;                            //从相机捕获一帧图像
		resize(frame,frame,videoSize);
		writer << frame;                         //将抓拍的图像写入AVI视频文件

		//resize(frame, imagedst, dsize);          //对捕捉的图像进行缩放操作
	    //namedWindow("FpgaLena Source Video", 1); //显示缩放后的双目视频
		//imshow("FpgaLena Source Video", imagedst);


		//cvtColor(frame, grayImage, CV_BGR2GRAY); //将双目源图像（彩色图）转为灰度数据
		//Canny(grayImage, grayImage, 10, 100, 3); //对图像进行Canny算子的图像边缘检测
		//namedWindow("Canny Video", 2);           //实时显示Canny边缘检测的效果图
		//imshow("Canny Video", grayImage);

		//if (waitKey(30) >= 0) break;
		
	}

	cap.release();                               //释放对相机的控制
	return 0;
}
