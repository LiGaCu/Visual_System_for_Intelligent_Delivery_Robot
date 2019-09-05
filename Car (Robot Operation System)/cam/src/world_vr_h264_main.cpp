#include <opencv2/opencv.hpp> 
//Linux下网络相关
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <string>
#include <sstream>
#include <iostream>

#include<thread>
#include<mutex>
#include<chrono>
#include<csignal>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

#include "../include/auto_vr_imageprocess.h"
#include "../include/VideoEncoder.h"

using namespace cv;
using namespace std;

//待传输图像默认大小为 640*480，可修改
#define IMG_WIDTH 640	// 需传输图像的宽
#define IMG_HEIGHT 360	// 需传输图像的高
//默认格式为CV_8UC3

#define endlen 17
#define framelen 10000
#define msglen 20
#define conmsglen 10

vector<int> param = { IMWRITE_JPEG_QUALITY, 70 };
vector<uchar> jpgbuff;
char Frameend[endlen] = "This is the end!";
uchar ultramsg[msglen];
uchar controlmsg[conmsglen];
int Netgood=1;

const char* keys =
{
	"{help h usage ? | | print this message}"
	"{@name | VideoTest | Video name, if not defined try to use VideoTest.avi}"
	"{source s | 0 | Video source, if not defined try to use cam0}"
};

double globalangle=90,autoangle=90;
double globalspeed=1500;

Mat frame,probuff,tranbuff;
//int proflag=0;
int safetimer;float line_rate;
mutex u1,u2;

void writeultramsgbuff(CVprocess* CVgo,uchar* msg);
void auto_drive(CVprocess* CVgo);
int sendmsg(int sockClient, vector<uchar> jpgbuff, uchar* msg);
int receivemsg(int sockClient, uchar* msg);
void receiveonly(int sockClient);
void safeguard(ros::Publisher pub_,geometry_msgs::Twist SpeedAndAngle,int servSock);
int main(int argc, char** argv)
{
    std::cout << "\n****************Linux Vision-based Control****************\n\n";
	signal(SIGPIPE,SIG_IGN);
	ros::init(argc, argv, "world_vr_h264");
	ros::NodeHandle n;
	ros::Publisher pub_= n.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);
	geometry_msgs::Twist SpeedAndAngle;
	
	controlmsg[0]=90;
	controlmsg[1]=(uchar)(1500 & 255);
	controlmsg[2]=(uchar)((1500>>8) & 255);
	controlmsg[3]=0;
	cv::CommandLineParser parser(argc, argv, keys);
	parser.about("No showing Detection!");
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}
    /******************************************/
    myselfencoder h264(640,360);
    if(h264.encoderinit()<0)
        return -1;
	/******************************************/
	VideoCapture cap;
	int realtime_frame = 0;
	cap.open(0);//RoadVideo.mp4");//
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);  //设置捕获视频的宽度
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);  //设置捕获视频的高度
	/*****************************************/
	//创建套接字
    int servSock = socket(AF_INET, SOCK_STREAM, 0);
	char IPV4[16];
	string ipv4buff = "120.79.6.109";
	int tmepi=0;
	for(;tmepi<ipv4buff.length();tmepi++)
		IPV4[tmepi]=ipv4buff[tmepi];
	IPV4[tmepi]=0;
	std::cout << "尝试连接服务器："<<IPV4;
	sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_addr.s_addr = inet_addr(IPV4);
	sockAddr.sin_port = htons(1234);

	/*****************************************/
	CVprocess CVgo(cap,Size(IMG_WIDTH,IMG_HEIGHT));
	cap>>frame;
	resize(frame, probuff, Size(IMG_WIDTH,IMG_HEIGHT));
	line_rate=1280/IMG_WIDTH;

	thread t1(auto_drive,&CVgo);
	t1.detach();
	thread t3(safeguard,pub_,SpeedAndAngle,servSock);
	t3.detach();
	while (true)
	{	
	    connect(servSock, (sockaddr*)&sockAddr, sizeof(sockAddr));
		Netgood=1;
		std::cout << "连接成功! Socket:"<< servSock<<"\n";

		thread t2(receiveonly,servSock);
        t2.detach();
		while(Netgood)
		{
			cap>>frame;
			
			resize(frame, frame, Size(IMG_WIDTH,IMG_HEIGHT));
			frame.copyTo(tranbuff);
            //resize(frame, tranbuff, Size(IMG_WIDTH/2,IMG_HEIGHT/2));
			//imencode(".jpeg", tranbuff, jpgbuff, param);
            
            u2.lock();
            writeultramsgbuff(&CVgo,ultramsg);
            u2.unlock();

            if (h264.bgr2yuv(tranbuff)==0 && h264.beginencode(&jpgbuff)==0)
			{
				ultramsg[msglen - 1] = h264.I_P_type;
				if (sendmsg(servSock, jpgbuff, ultramsg) == -1)
					Netgood = 0;
			}

			u1.lock();
			frame.copyTo(probuff);
			u1.unlock();

			if(controlmsg[3]==1)
				globalangle=autoangle;
			else
				globalangle=controlmsg[0];
			
			globalspeed=(double)(controlmsg[1]) +(double)(controlmsg[2])*256;
			SpeedAndAngle.linear.x=globalspeed;
			SpeedAndAngle.angular.z=globalangle;
			pub_.publish(SpeedAndAngle); 
			ros::spinOnce();

            this_thread::sleep_for(chrono::milliseconds(10));
			this_thread::yield();
			//cout<<"Speed: "<<globalspeed<<"	Angle: "<<globalangle<<"\n";
		}
		SpeedAndAngle.linear.x=1500;
		SpeedAndAngle.angular.z=90;
		pub_.publish(SpeedAndAngle); 
		ros::spinOnce();
		std::cout << "\n*** 注意！！！***\n";
		std::cout << "\n用户掉线! Socket:" << servSock<<"\n\n    停车！！！";
		std::cout << "\n\n****************等待重新连接 ****************\n\n";
		shutdown(servSock, SHUT_RDWR); 
		close(servSock);
		servSock = socket(AF_INET, SOCK_STREAM, 0);
	}
	
	SpeedAndAngle.linear.x=1500;
	SpeedAndAngle.angular.z=90;
	pub_.publish(SpeedAndAngle); 
	ros::spinOnce();
	
	cap.release();
	close(servSock);
	return 0;
}

void receiveonly(int sockClient)
{
    while(Netgood==1)
    {
        if(receivemsg(sockClient, controlmsg)==-1)
	    	Netgood=0;
		safetimer=0;
    }
}
void safeguard(ros::Publisher pub_,geometry_msgs::Twist SpeedAndAngle,int servSock)
{
	while(true)
	{
		if(Netgood==1)
		{
			safetimer++;
			if(safetimer>10)
			{
				safetimer=0;
				SpeedAndAngle.linear.x=1500;
				SpeedAndAngle.angular.z=90;
				pub_.publish(SpeedAndAngle); 
				ros::spinOnce();
			}
			// if(safetimer>100)
			// {
				// safetimer=0;
				// shutdown(servSock, SHUT_RDWR); 
				// Netgood=0;
			// }
		}
		this_thread::sleep_for(chrono::milliseconds(100));
		this_thread::yield();
	}
}
void auto_drive(CVprocess* CVgo)
{
	while(true)
	{
		u1.lock();
		probuff.copyTo(CVgo->imageBGR);
		u1.unlock();
		CVgo->processing();
		u2.lock();
        CVgo->confirmlineangle();
		u2.unlock();
		autoangle=CVgo->angle;
	}
}

/*msg最后一位不得写入，留给图象压缩后标记帧类型，以便接收方识别*/
void writeultramsgbuff(CVprocess* CVgo,uchar* msg)
{
	msg[0]=CVgo->havefoundL_flag;
	msg[1]=((int)(CVgo->lineLR[0][0]*line_rate))&255;
	msg[2]=((int)(CVgo->lineLR[0][0]*line_rate)>>8)&255;
	msg[3]=((int)(CVgo->lineLR[0][1]*line_rate))&255;
	msg[4]=((int)(CVgo->lineLR[0][1]*line_rate)>>8)&255;
	msg[5]=((int)(CVgo->lineLR[0][2]*line_rate))&255;
	msg[6]=((int)(CVgo->lineLR[0][2]*line_rate)>>8)&255;
	msg[7]=((int)(CVgo->lineLR[0][3]*line_rate))&255;
	msg[8]=((int)(CVgo->lineLR[0][3]*line_rate)>>8)&255;
	msg[9]=CVgo->havefoundR_flag;
	msg[10]=((int)(CVgo->lineLR[1][0]*line_rate))&255;
	msg[11]=((int)(CVgo->lineLR[1][0]*line_rate)>>8)&255;
	msg[12]=((int)(CVgo->lineLR[1][1]*line_rate))&255;
	msg[13]=((int)(CVgo->lineLR[1][1]*line_rate)>>8)&255;
	msg[14]=((int)(CVgo->lineLR[1][2]*line_rate))&255;
	msg[15]=((int)(CVgo->lineLR[1][2]*line_rate)>>8)&255;
	msg[16]=((int)(CVgo->lineLR[1][3]*line_rate))&255;
	msg[17]=((int)(CVgo->lineLR[1][3]*line_rate)>>8)&255;
	msg[18]=(int)(CVgo->angle);
}

int sendmsg(int sockClient, vector<uchar> jpgbuff, uchar* msg)
{
	long int num = 0, sentnum = 0;

	for (int i = 0; i < msglen; i++)
		jpgbuff.push_back((uchar)(msg[i]));

	for (int i = 0; i < endlen; i++)
		jpgbuff.push_back((uchar)(Frameend[i]));

	size_t ojpgsize = jpgbuff.size();
	while (sentnum < ojpgsize)
	{
		if (ojpgsize - sentnum<framelen)
			num = send(sockClient, (char *)(jpgbuff.data() + sentnum), ojpgsize - sentnum, 0);
		else
			num = send(sockClient, (char *)(jpgbuff.data() + sentnum), framelen, 0);
		if (num == -1)
			return -1;
		sentnum += num;
	}
	return 0;
}

int receivemsg(int sockClient, uchar* msg)
{
	int num = 0, recvnum = 0;
	while (recvnum < conmsglen)
	{
		num = recv(sockClient, (char *)(msg + recvnum), conmsglen - recvnum, 0);
		recvnum += num;
		if (num == -1|| num ==0 )
			return -1;
	}
	return 0;
}

