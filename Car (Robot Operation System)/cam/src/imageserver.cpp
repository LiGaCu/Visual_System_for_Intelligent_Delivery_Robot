#include "opencv2/opencv.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
//待传输图像默认大小为 640*480，可修改
#define IMG_WIDTH 640	// 需传输图像的宽
#define IMG_HEIGHT 260	// 需传输图像的高
//默认格式为CV_8UC3

#define endlen 17
#define framelen 10000
#define msglen 10

using namespace cv;
using namespace std;

vector<int> param = { IMWRITE_JPEG_QUALITY, 70 };
vector<uchar> jpgbuff;

char Frameend[endlen] = "This is the end!";
uchar ultramsg[msglen];
uchar controlmsg[msglen];

int sendmsg(int sockClient, vector<uchar> jpgbuff, uchar* msg);
int receivemsg(int sockClient, uchar* msg);

int main(int, char**)
{
	VideoCapture cap(0); // open the default camera
	cap.set(CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
	cap.set(CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
	cap.set(CAP_PROP_FPS, 30);
	if (!cap.isOpened())  // check if we succeeded
		return -1;
	Mat frame;
	//创建套接字
	//win//SOCKET servSock = ::socket(AF_INET, SOCK_STREAM, 0);
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
	//接收客户端请求
	while(true)
	{
		int connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
		std::cout << "\n新用户接入! Socket:"<< connfd<<"\n";
		while (1)
		{
			cap >> frame; // get a new frame from camera
			//DWORD clk1 = GetTickCount();
			imencode(".jpeg", frame, jpgbuff, param);
			if (sendmsg(connfd, jpgbuff, ultramsg) == -1)
				break;
			if(receivemsg(connfd, controlmsg)==-1)
				break;
			cout << "Angle:" << (int)(controlmsg[0]) << "	Speed:" << (int)(controlmsg[1] * 10) << "\n";
			//DWORD clk2 = GetTickCount();
			//cout << "Go and Return time: " << clk2 - clk1 << " ms" << "	\n";
			//Sleep(28);
		}
		cout << "\n*** 注意！！！***\n";
		std::cout << "\n用户掉线! Socket:" << connfd<<"\n\n    停车！！！";
		cout << "\n\n****************等待重新连接 ****************\n\n";
		close(connfd);

	}
	// the camera will be deinitialized automatically in VideoCapture destructor

	//关闭套接字
	close(listenfd);

	//终止 DLL 的使用
	return 0;
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
	while (recvnum < msglen)
	{
		num = recv(sockClient, (char *)(msg + recvnum), msglen - recvnum, 0);
		recvnum += num;
		if (num == -1|| num ==0 )
			return -1;
	}
	return 0;
}
