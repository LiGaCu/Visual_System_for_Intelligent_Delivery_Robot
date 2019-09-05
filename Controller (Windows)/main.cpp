#include <opencv2/opencv.hpp>
#include <WinSock2.h>
#include <Windows.h>
#include <thread>
#include <chrono>
#include <conio.h>
#include <mutex>
#include<algorithm>
#include "VideoDecoder.h"
#include "CNN_Yolo3.h"

#pragma comment (lib, "ws2_32.lib")
#pragma warning(disable : 4996)

using namespace cv;
using namespace std;

//待传输图像默认大小为 640*360，可修改
#define IMG_WIDTH 1280	// 需传输图像的宽
#define IMG_HEIGHT 720	// 需传输图像的高
//默认格式为CV_8UC3

#define Hlimit1 1/3
#define Vlimit1 1/4
#define Hlimit2 2/3
#define Vlimit2 3/4

#define endlen 17
#define framelen 10000
#define msglen 20
#define conmsglen 10

char Frameend[endlen] = "This is the end!";
char ultramsg[msglen];
uchar controlmsg[conmsglen];
vector<uchar> framejpg;
Mat frame;// (IMG_WIDTH, IMG_HEIGHT, CV_8UC3);
int runstill = 1;

#define anglecenter 90
#define speedcenter 1500
int Anglebuff = 90, Speedbuff = 1500, Movemode = 0;//Movemode为0时，为手动模式；1时，为自动模式
int TLpressed = 0; int SUpressed = 0;
int TRpressed = 0; int SDpressed = 0;
int ShowLine = 0;
VideoWriter writer;
int VWnum = 0, VWneedopen = 0, VWrun = 0;
int CNNrun; int ShowDetect = 0,CNNneedclose = 0; Mat CNNframe; mutex u1;
myselfyolo yolo;
//DWORD codetime = 0;
char delaytest[50];

void welcomeinfo();
int recievejpg(SOCKET sockServer, vector<uchar>* jpgbuff);
int sendmsg(SOCKET sockClient, uchar* msg);
int tellend(vector<uchar> srcvector, const char* goal);
void deleteend(vector<uchar>* fixbuff, const char* goal);
//void keyonly(void);
void ChangeValue(void);
void VideoRe(Mat src);
void drawultramsg(Mat dst);
void drawicon(Mat dst);
void CNNyolo(void);
int main()
{
	welcomeinfo();
	myselfdecoder h264;
	if (h264.decoderinit()<0)
		return -1;
	yolo.yoloinit();
	//初始化 DLL
	WSADATA data;
	WORD w = MAKEWORD(2, 0);
	WSAStartup(w, &data);
	// 创建套接字
	SOCKET s;
	s = socket(AF_INET, SOCK_STREAM, 0);

	cout << "\nPlease enter server IP address（请输入服务器IPV4地址）:";
	char IPV4[16];
	cin >> IPV4;
	cout << "\nConnecting ...（连接中……）\n";

	// 构造ip地址
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.S_un.S_addr = inet_addr(IPV4);
	addr.sin_port = htons(2465);

	controlmsg[0] = anglecenter;
	controlmsg[1] = speedcenter & 255;
	controlmsg[2] = (speedcenter / 256) & 255;
	//	thread scankey(keyonly);
	//	scankey.detach();
	thread button(ChangeValue);	
	button.detach();

	while (runstill)
	{
		::connect(s, (sockaddr*)&addr, sizeof(addr));
		std::cout << "\nLinked! Ready to be controlled!（已建立连接！操控准备就绪！）\n";
		namedWindow("Imgwindow", WINDOW_AUTOSIZE);
		sendmsg(s, controlmsg);

		CNNrun = 1;
		thread t1(CNNyolo);
		while (runstill)
		{
			
			framejpg.clear();
			if (recievejpg(s, &framejpg) == -1)	
				break;

			deleteend(&framejpg, Frameend);	
			//frame = imdecode(framejpg, IMREAD_COLOR);
//			DWORD clk1 = GetTickCount();
			if (h264.begindecode(&framejpg, frame) < 0)
				return -1;
//			DWORD clk2 = GetTickCount();
//			cout << "Deconding took: " << clk2 - clk1 << "ms\n";
//			resize(frame, frame, Size(IMG_WIDTH, IMG_HEIGHT));
			u1.lock();
			frame.copyTo(CNNframe);
			u1.unlock();

			VideoRe(frame);
			if (ShowLine == 1)
				drawultramsg(frame);

			resize(frame, frame, Size(IMG_WIDTH, IMG_HEIGHT));

			drawicon(frame);

			//codetime = (uchar)(ultramsg[msglen - 2])+((uchar)(ultramsg[msglen - 3])<<8);
			//sprintf(delaytest, "Frame shooted at:%d ms", codetime);
			//if (frame.data)  putText(frame, delaytest, Point(800, 700), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 100), 2, LINE_8, 0);

			if (frame.data) imshow("Imgwindow", frame);
			waitKey(1);
			if (sendmsg(s, controlmsg) == -1)
				break;
			//cout << "Angle:" << (int)(controlmsg[0]) << "	Speed:" << (int)(controlmsg[1] * 10) << "\n";

		}
		closesocket(s);
		s = socket(AF_INET, SOCK_STREAM, 0);
		CNNrun = 0;
		t1.join();
		destroyWindow("Imgwindow");
		this_thread::sleep_for(chrono::milliseconds(100));
		this_thread::yield();
		cout << "网络错误！！！尝试重连中！！！";
	}

	::WSACleanup();
	frame.release();
	if(writer.isOpened())
		writer.release();
	return 0;
}

void welcomeinfo()
{
	cout << "\n***************************************************************************\n";
	cout << "--------Windows Low-delay Vision-based Controller ( By Li Jiatong )--------\n";
	cout << "---------------------低延迟视觉控制操控端 作者：李佳桐---------------------\n\n";
}
vector<uchar> restbuff;
int recievejpg(SOCKET sockServer, vector<uchar>* jpgbuff)
{
	int num = 0;

	vector<uchar> samllbuff;
	size_t smallbuffbeginpos;
	int endlastpos = -1;

	char unitbuff[framelen];

	if (!restbuff.empty())
	{
		samllbuff = restbuff;
		endlastpos = tellend(restbuff, Frameend);
		if (endlastpos < 0)
		{
			for (vector<uchar>::iterator it = restbuff.begin(); it < restbuff.end(); it++)
				jpgbuff->push_back(*it);
			restbuff.clear();
		}
		else
		{
			for (vector<uchar>::iterator it = restbuff.begin(); it < restbuff.begin() + endlastpos + 1; it++)
				jpgbuff->push_back(*it);
			restbuff.erase(restbuff.begin(), restbuff.begin() + endlastpos + 1);
			return 0;
		}
	}

	while (endlastpos < 0)
	{
		if (samllbuff.size() > endlen)
			samllbuff.erase(samllbuff.begin(), samllbuff.end() - endlen);
		smallbuffbeginpos = samllbuff.size();

		num = recv(sockServer, (char *)(unitbuff), framelen, 0);

		if (num == SOCKET_ERROR || num == 0)
			return -1;

		for (int i = 0; i < num; i++)
			samllbuff.push_back((uchar)(unitbuff[i]));

		endlastpos = tellend(samllbuff, Frameend);
		if (endlastpos < 0)
		{
			for (vector<uchar>::iterator it = samllbuff.begin() + smallbuffbeginpos; it < samllbuff.end(); it++)
				jpgbuff->push_back(*it);
		}
		else
		{
			for (vector<uchar>::iterator it = samllbuff.begin() + smallbuffbeginpos; it < samllbuff.begin() + endlastpos + 1; it++)
				jpgbuff->push_back(*it);
			for (vector<uchar>::iterator it = samllbuff.begin() + endlastpos + 1; it < samllbuff.end(); it++)
				restbuff.push_back(*it);
		}
	}
	return 0;
}

int sendmsg(SOCKET sockClient, uchar* msg)
{
	int num = 0, recvnum = 0;
	while (recvnum < conmsglen)
	{
		num = send(sockClient, (char *)(msg + recvnum), conmsglen - recvnum, 0);
		if (num == SOCKET_ERROR)
			return -1;
		recvnum += num;
	}
	return 0;
}

int tellend(vector<uchar> srcvector, const char* goal)
{
	for (vector<uchar>::iterator it = srcvector.begin(); it < srcvector.end() - endlen + 1; it++)
	{
		if (*it == (uchar)(*goal))
		{
			for (int i = 1; i < endlen; i++)
			{
				if (*(it + i) != (uchar)(*(goal + i)))
					break;

				if (i == endlen - 1)
					return (int)(it - srcvector.begin() + i);
			}
		}
	}
	return -1;
}


void deleteend(vector<uchar>* fixbuff, const char* goal)
{
	for (int i = 0; i<endlen; i++)
	{
		if (fixbuff->at(fixbuff->size() - 1 - i) != (uchar)(*(goal + endlen - 1 - i)))
			break;
		if (i == endlen - 1)
		{
			fixbuff->erase(fixbuff->end() - endlen, fixbuff->end());
			for (int j = 0; j <msglen; j++)
			{
				ultramsg[msglen - 1 - j] = fixbuff->at(fixbuff->size() - 1 - j);
			}
			fixbuff->erase(fixbuff->end() - msglen, fixbuff->end());
		}
	}
}

//char s;
//int lastperiod;
//void keyonly()
//{
//	while (1)
//	{
//		s = getch();
//		if (s == 'a' || s == 'd')
//			lastperiod = 20;
//		else
//			lastperiod = 1;
//		//this_thread::sleep_for(chrono::milliseconds(5));
//		//this_thread::yield();
//	}
//}

#define keydown(vk_code) ((GetAsyncKeyState(vk_code)&0x8000)?1:0)
void ChangeValue()
{
	while (1)
	{
		if (keydown('A'))
		{
			if (Anglebuff < anglecenter)
				Anglebuff += 8;
			else
				Anglebuff += 4;
			if (Anglebuff > 150)
				Anglebuff = 150;
			TLpressed = 1; TRpressed = 0;
			Movemode = 0;
		}
		else if (keydown('D'))
		{
			if (Anglebuff > anglecenter)
				Anglebuff -= 8;
			else
				Anglebuff -= 4;
			if (Anglebuff < 30)
				Anglebuff = 30;
			TLpressed = 0; TRpressed = 1;
			Movemode = 0;
		}
		else
		{
			if (Anglebuff > anglecenter)
				Anglebuff -= 4;
			else if (Anglebuff < anglecenter)
				Anglebuff += 4;
			TLpressed = 0; TRpressed = 0;
		}
		controlmsg[0] = (uchar)Anglebuff & 255;


		if (keydown('W'))
		{
			if (keydown(VK_SHIFT))//加速前进
			{
				Speedbuff += 1;
				if (Speedbuff > 1700)
					Speedbuff = 1700;
			}//前进
			controlmsg[1] = (uchar)Speedbuff & 255;
			controlmsg[2] = (uchar)(Speedbuff / 256) & 255;
			SUpressed = 1; SDpressed = 0;
		}
		else if (keydown('S'))
		{
			if (keydown(VK_SHIFT))//倒车
			{
				controlmsg[1] = (uchar)1200 & 255;
				controlmsg[2] = (uchar)(1200 / 256) & 255;
				SUpressed = 0; SDpressed = 1;
			}
			else//减速前进
			{
				Speedbuff -= 1;
				if (Speedbuff < 1500)
					Speedbuff = 1500;
				controlmsg[1] = (uchar)Speedbuff & 255;
				controlmsg[2] = (uchar)(Speedbuff / 256) & 255;
				SUpressed = 1; SDpressed = 0;
			}
		}
		else
		{
			if (Movemode == 0)
			{
				controlmsg[1] = (uchar)speedcenter & 255;
				controlmsg[2] = (uchar)(speedcenter / 256) & 255;
				SUpressed = 0; SDpressed = 0;
			}
		}

		if (keydown(VK_TAB) && keydown('Q'))
			Movemode = 1;
		controlmsg[3] = (uchar)(Movemode);

		if (keydown('L'))
		{
			if (keydown(VK_SHIFT))
				ShowLine = 0;
			else
				ShowLine = 1;
		}

		if (keydown('R'))
		{
			if (keydown(VK_SHIFT))
				VWrun = 0;
			else if (VWrun == 0)
			{
				VWneedopen = 1;
				VWrun = 1;
			}
		}

		if (keydown('O'))
		{
			if (keydown(VK_SHIFT))
			{
				ShowDetect = 0;
				CNNneedclose = 1;
			}
			else
				ShowDetect = 1;
		}

		if (keydown(VK_ESCAPE))
			runstill = 0;
		this_thread::sleep_for(chrono::milliseconds(20));
		this_thread::yield();
	}
}

char VWnumchar[5];
string videoname;
const char* name;
void VideoRe(Mat src)
{
	if (VWneedopen)
	{
		VWneedopen = 0; VWnum++;
		sprintf(VWnumchar, "%d", VWnum);
		videoname = VWnumchar;
		videoname += "TranVideo.avi";
		name = videoname.data();
		writer.open(name, VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(src.cols, src.rows));
	}
	if (VWrun)
		writer << src;
}

void drawultramsg(Mat dst)
{
	int Line_x_1, Line_y_1, Line_x_2, Line_y_2;
	if (ultramsg[0] == 1)
	{
		Line_x_1 = (int)((uchar)(ultramsg[1]) + (uchar)(ultramsg[2]) * 256);
		Line_y_1 = (int)((uchar)(ultramsg[3]) + (uchar)(ultramsg[4]) * 256);
		Line_x_2 = (int)((uchar)(ultramsg[5]) + (uchar)(ultramsg[6]) * 256);
		Line_y_2 = (int)((uchar)(ultramsg[7]) + (uchar)(ultramsg[8]) * 256);
		line(dst, Point(Line_x_1, Line_y_1), Point(Line_x_2, Line_y_2), Scalar(0, 255, 0), 3, LINE_8);
	}
	if (ultramsg[9] == 1)
	{
		Line_x_1 = (int)((uchar)(ultramsg[10]) + (uchar)(ultramsg[11]) * 256);
		Line_y_1 = (int)((uchar)(ultramsg[12]) + (uchar)(ultramsg[13]) * 256);
		Line_x_2 = (int)((uchar)(ultramsg[14]) + (uchar)(ultramsg[15]) * 256);
		Line_y_2 = (int)((uchar)(ultramsg[16]) + (uchar)(ultramsg[17]) * 256);
		line(dst, Point(Line_x_1, Line_y_1), Point(Line_x_2, Line_y_2), Scalar(0, 255, 0), 3, LINE_8);
	}
}

void drawicon(Mat dst)
{
	Mat temp1(dst.rows*Hlimit1, dst.cols*Vlimit1, CV_8UC3, Scalar(243, 217, 153));//绘制图标模板
	Mat temp2;
	Point triangle3[3] = { Point(temp1.cols * 1 / 3,temp1.rows / 2),Point(temp1.cols * 2 / 3,temp1.rows * 1 / 3),Point(temp1.cols * 2 / 3,temp1.rows * 2 / 3) };
	Point triangle4[3] = { Point(temp1.cols * 2 / 3,temp1.rows / 2),Point(temp1.cols * 1 / 3,temp1.rows * 1 / 3),Point(temp1.cols * 1 / 3,temp1.rows * 2 / 3) };
	if (TLpressed == 1)
	{
		cv::fillConvexPoly(temp1, triangle3, 3, Scalar(204, 72, 63), 8);
		temp2 = dst(Rect(0, dst.rows*Hlimit2 / 2, dst.cols*Vlimit1, dst.rows*Hlimit1));
		cv::addWeighted(temp1, 0.2, temp2, 0.8, 0, temp2);
	}
	else if (TRpressed == 1)
	{
		cv::fillConvexPoly(temp1, triangle4, 3, Scalar(204, 72, 63), 8);
		temp2 = dst(Rect(dst.cols*Vlimit2, dst.rows*Hlimit2 / 2, dst.cols*Vlimit1, dst.rows*Hlimit1));
		cv::addWeighted(temp1, 0.2, temp2, 0.8, 0, temp2);
	}

	Mat temp3(dst.rows*Hlimit1, dst.cols*Vlimit1, CV_8UC3, Scalar(243, 217, 153));//绘制图标模板
	Point triangle1[3] = { Point(temp1.cols / 2,temp1.rows / 3),Point(temp1.cols * 1 / 3,temp1.rows * 2 / 3),Point(temp1.cols * 2 / 3,temp1.rows * 2 / 3) };
	Point triangle2[3] = { Point(temp1.cols / 2,temp1.rows * 2 / 3),Point(temp1.cols * 1 / 3,temp1.rows * 1 / 3),Point(temp1.cols * 2 / 3,temp1.rows * 1 / 3) };
	if (SUpressed == 1)
	{
		cv::fillConvexPoly(temp3, triangle1, 3, Scalar(204, 72, 63), 8);
		temp2 = dst(Rect(dst.cols*Vlimit2 / 2, 0, dst.cols*Vlimit1, dst.rows*Hlimit1));
		cv::addWeighted(temp3, 0.2 + (double)(Speedbuff - speedcenter) / 1000, temp2, 0.8 - (double)(Speedbuff - speedcenter) / 1000, 0, temp2);
	}
	else if (SDpressed == 1)
	{
		cv::fillConvexPoly(temp3, triangle2, 3, Scalar(204, 72, 63), 8);
		temp2 = dst(Rect(dst.cols*Vlimit2 / 2, dst.rows*Hlimit2 - 1, dst.cols*Vlimit1, dst.rows*Hlimit1));
		cv::addWeighted(temp3, 0.2, temp2, 0.8, 0, temp2);
	}

	char textshow[50];
	double textscale = (double)(dst.rows) / 720;
	int linesize = dst.rows * 2 / 720;
	int text_x_loc = dst.rows * 10 / 720;
	int text_y_loc = dst.rows * 40 / 720;
	int circle_r = dst.rows * 10 / 720;
	if (Movemode == 0)
	{
		if ((uchar)(controlmsg[0]) > anglecenter)
			sprintf(textshow, "TurnAngle:L %2d'    SetSpeed:%4d", ((uchar)(controlmsg[0]) - 90), (int)((uchar)(controlmsg[1]) + (uchar)(controlmsg[2]) * 256));
		else if ((uchar)(controlmsg[0]) < anglecenter)
			sprintf(textshow, "TurnAngle:R %2d'    SetSpeed:%4d", (90 - (uchar)(controlmsg[0])), (int)((uchar)(controlmsg[1]) + (uchar)(controlmsg[2]) * 256));
		else
			sprintf(textshow, "TurnAngle:S %2d'    SetSpeed:%4d", 0, (int)((uchar)(controlmsg[1]) + (uchar)(controlmsg[2]) * 256));
		putText(dst, textshow, Point(text_x_loc, text_y_loc), FONT_HERSHEY_SIMPLEX, textscale, Scalar(255, 255, 100), linesize, LINE_8, 0);
		putText(dst, "Mode: Human Control", Point(text_x_loc, text_y_loc * 2), FONT_HERSHEY_SIMPLEX, textscale, Scalar(255, 255, 100), linesize, LINE_8, 0);
	}
	else
	{
		if ((uchar)(ultramsg[18]) > anglecenter)
			sprintf(textshow, "TurnAngle:L %2d'    SetSpeed:%4d", (uchar)(ultramsg[18]) - 90, (int)((uchar)(controlmsg[1]) + (uchar)(controlmsg[2]) * 256));
		else if ((uchar)(ultramsg[18]) < anglecenter)
			sprintf(textshow, "TurnAngle:R %2d'    SetSpeed:%4d", (90 - (uchar)(ultramsg[18])), (int)((uchar)(controlmsg[1]) + (uchar)(controlmsg[2]) * 256));
		else
			sprintf(textshow, "TurnAngle:S %2d'    SetSpeed:%4d", 0, (int)((uchar)(ultramsg[18]) + (uchar)(controlmsg[2]) * 256));
		putText(dst, textshow, Point(text_x_loc, text_y_loc), FONT_HERSHEY_SIMPLEX, textscale, Scalar(255, 255, 100), linesize, LINE_8, 0);
		putText(dst, "Mode: Auto Driving", Point(text_x_loc, text_y_loc * 2), FONT_HERSHEY_SIMPLEX, textscale, Scalar(68, 68, 255), linesize, LINE_8, 0);
	}

	if (VWrun)
		circle(dst, Point(dst.cols - circle_r * 2, circle_r * 2), circle_r, Scalar(0, 0, 255), -1, LINE_8);
	else
		circle(dst, Point(dst.cols - circle_r * 2, circle_r * 2), circle_r, Scalar(0, 0, 255), linesize, LINE_8);
}

void CNNyolo(void)
{
	while (CNNrun)
	{
		if (!CNNframe.empty()&& ShowDetect)
		{
			Mat temp;
			u1.lock();
			CNNframe.copyTo(temp);
			u1.unlock();
			resize(temp, temp, Size(640, 360));
			DWORD clk1 = GetTickCount();
			yolo.yoloprocess(temp);
			DWORD clk2 = GetTickCount();
			sprintf(delaytest, "Inference took:%d ms", clk2- clk1);
			putText(temp,delaytest,Point(0, 350), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(255, 255, 100), 2, LINE_8, 0);
			imshow("CNN Detection", temp);
			waitKey(10);
		}
		else
		{
			this_thread::sleep_for(chrono::milliseconds(30));
			this_thread::yield();
			if (CNNneedclose == 1)
				destroyWindow("CNN Detection");
		}
	}
	destroyWindow("CNN Detection");
}