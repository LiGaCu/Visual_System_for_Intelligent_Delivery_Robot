#include <opencv2/opencv.hpp> 
//Linux下网络相关
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include <string>
#include <sstream>
#include <iostream>

#include<thread>
#include<mutex>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

#include "../include/auto_vr_imageprocess.h"

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

Mat frame,probuff;
int proflag=0;
mutex u1,u2;

void writeultramsgbuff(CVprocess* CVgo,uchar* msg);
void auto_drive(CVprocess* CVgo);
int sendmsg(int sockClient, vector<uchar> jpgbuff, uchar* msg);
int receivemsg(int sockClient, uchar* msg);

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
	/******************************************/
	VideoCapture cap;
	int realtime_frame = 0;
	cap.open(0);//RoadVideo.mp4");//
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);  //设置捕获视频的宽度
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);  //设置捕获视频的高度
	/*****************************************/
		int listenfd;
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
	/*char IPV4[16];
	std::cin >> IPV4;*/
	//绑定套接字
	struct sockaddr_in sockAddr;
	
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	sockAddr.sin_port = htons(1234);
	bind(listenfd, (struct sockaddr*)&sockAddr, sizeof(sockAddr));

	cout << "\n****************Linux Vision-based Control****************\n\n";

	//进入监听状态
	listen(listenfd, 5);
	std::cout << "listening\n";
	/*****************************************/
	CVprocess CVgo(cap,Size(IMG_WIDTH,IMG_HEIGHT));
	cap>>frame;
	resize(frame, probuff, Size(IMG_WIDTH,IMG_HEIGHT));
	thread t(auto_drive,&CVgo);
	t.detach();

	while (true)
	{	
		int connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
		std::cout << "\n新用户接入! Socket:"<< connfd<<"\n";
		Netgood=1;
		while(Netgood)
		{
			cap>>frame;
			resize(frame, frame, Size(IMG_WIDTH,IMG_HEIGHT));
			imencode(".jpeg", frame, jpgbuff, param);

			u2.lock();
			writeultramsgbuff(&CVgo,ultramsg);
			u2.unlock();
			
			if (sendmsg(connfd, jpgbuff, ultramsg) == -1)
				Netgood=0;

			u1.lock();
			frame.copyTo(probuff);
			u1.unlock();

			if(receivemsg(connfd, controlmsg)==-1)
				Netgood=0;

			if(controlmsg[3]==1)
				globalangle=autoangle;
			else
				globalangle=controlmsg[0];

			globalspeed=(double)(controlmsg[1]) +(double)(controlmsg[2])*256;
			SpeedAndAngle.linear.x=globalspeed;
			SpeedAndAngle.angular.z=globalangle;
			pub_.publish(SpeedAndAngle); 
			ros::spinOnce();
			//cout<<"Speed: "<<globalspeed<<"	Angle: "<<globalangle<<"\n";
		}
		SpeedAndAngle.linear.x=1500;
		SpeedAndAngle.angular.z=90;
		pub_.publish(SpeedAndAngle); 
		ros::spinOnce();
		cout << "\n*** 注意！！！***\n";
		std::cout << "\n用户掉线! Socket:" << connfd<<"\n\n    停车！！！";
		cout << "\n\n****************等待重新连接 ****************\n\n";
		close(connfd);
	}
	
	SpeedAndAngle.linear.x=1500;
	SpeedAndAngle.angular.z=90;
	pub_.publish(SpeedAndAngle); 
	ros::spinOnce();
	
	cap.release();
	close(listenfd);
	return 0;
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
void writeultramsgbuff(CVprocess* CVgo,uchar* msg)
{
	msg[0]=CVgo->havefoundL_flag;
	msg[1]=CVgo->lineLR[0][0]&255;
	msg[2]=(int)(CVgo->lineLR[0][0]/256)&255;
	msg[3]=CVgo->lineLR[0][1]&255;
	msg[4]=(int)(CVgo->lineLR[0][1]/256)&255;
	msg[5]=CVgo->lineLR[0][2]&255;
	msg[6]=(int)(CVgo->lineLR[0][2]/256)&255;
	msg[7]=CVgo->lineLR[0][3]&255;
	msg[8]=(int)(CVgo->lineLR[0][3]/256)&255;
	msg[9]=CVgo->havefoundR_flag;
	msg[10]=CVgo->lineLR[1][0]&255;
	msg[11]=(int)(CVgo->lineLR[1][0]/256)&255;
	msg[12]=CVgo->lineLR[1][1]&255;
	msg[13]=(int)(CVgo->lineLR[1][1]/256)&255;
	msg[14]=CVgo->lineLR[1][2]&255;
	msg[15]=(int)(CVgo->lineLR[1][2]/256)&255;
	msg[16]=CVgo->lineLR[1][3]&255;
	msg[17]=(int)(CVgo->lineLR[1][3]/256)&255;
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

